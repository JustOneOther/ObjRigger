#[cfg(not(feature = "debug"))]
use std::{ env::args, fs::File, io::Write, thread::sleep, time::Duration };
#[cfg(feature = "debug")]
use std::env::vars;
use std::path::PathBuf;
use nalgebra::{Rotation3, Vector3};
use tobj::LoadOptions;


const FLOATING_TOLERANCE: f64 = 1e-10;


#[derive(Clone)]
struct PointDist {
	point: Vector3<f64>,
	dist: f64,
}


fn main() {
	#[cfg(not(feature = "debug"))]
	let args = args().collect::<Vec<String>>();
	#[cfg(not(feature = "debug"))]
	let (obj, reference) = if args.len() < 3 {
		let obj = rfd::FileDialog::new()
			.add_filter("Wavefront .obj", &["obj"])
			.set_title("Obj to move")
			.pick_file();
		if obj.is_none() { return }

		let reference = rfd::FileDialog::new()
			.add_filter("Wavefront .obj", &["obj"])
			.set_title("Reference obj")
			.pick_file();
		if reference.is_none() { return }

		(obj.unwrap(), reference.unwrap())
	} else {
		(PathBuf::from(args[1].clone()), PathBuf::from(args[2].clone()))
	};
	#[cfg(feature = "debug")]
	let (obj, reference) = (PathBuf::from(vars().find(|x| x.0 == "target").unwrap().1), PathBuf::from(vars().find(|x| x.0 == "ref").unwrap().1));

	let obj_data = match tobj::load_obj(obj, &LoadOptions::default()) {
		Ok(o) => o.0,
		Err(err) => {
			println!("Failed to load target .obj: {}", err);
			return
		}
	};
	let ref_data = match tobj::load_obj(reference, &LoadOptions::default()) {
		Ok(o) => o.0,
		Err(err) => {
			println!("Failed to load reference .obj: {}", err);
			return
		}
	};

	let mut output = String::new();

	for model in &obj_data {
		let ref_model = match ref_data.iter().find(|x| x.name == model.name) {
			Some(m) => m,
			None => {
				println!("Unable to find partner for model {}", model.name);
				continue
			}
		};

		if model.mesh.positions.len() == 0 || ref_model.mesh.positions.len() == 0 {
			println!("Model \"{}\" has no vertices", model.name);
			continue
		}
		if model.mesh.positions.len() != ref_model.mesh.positions.len() {
			println!("Model \"{}\" does not have as many vertices as its reference", model.name);
			continue
		}

		let mut model_verts: Vec<Vector3<f64>> = Vec::with_capacity(model.mesh.positions.len() / 3);
		let mut ref_verts: Vec<Vector3<f64>> = Vec::with_capacity(model.mesh.positions.len() / 3);

		for i in (0..model.mesh.positions.len()).step_by(3) {
			model_verts.push(Vector3::new(model.mesh.positions[i] as f64, model.mesh.positions[i + 1] as f64, model.mesh.positions[i + 2] as f64));
			ref_verts.push(Vector3::new(ref_model.mesh.positions[i] as f64, ref_model.mesh.positions[i + 1] as f64, ref_model.mesh.positions[i + 2] as f64));
		}

		let (ref_origin, global_angle, global_transform): (Vector3<f64>, (f64, f64, f64), Rotation3<f64>) = if !(get_avg_delta(&model_verts, &ref_verts) < FLOATING_TOLERANCE) {
			let model_centroid = get_avg_point(&model_verts);
			let ref_centroid = get_avg_point(&ref_verts);

			let model_local_points = get_point_dists(&model_verts, &model_centroid);
			let ref_local_points = get_point_dists(&ref_verts, &ref_centroid);

			let local_transform = match match_planar_rotation(model_local_points, ref_local_points) {
				Some(A) => A,
				None => {
					println!("Unable to construct transformation matrix for model {}\n", model.name);
					continue
				}
			};

			let ref_origin = ref_centroid - local_transform * model_centroid;

			let model_origin_points = get_point_dists(&model_verts, &Vector3::new(0f64, 0f64, 0f64));
			let ref_origin_points = get_point_dists(&ref_verts, &ref_origin);

			let global_transform = match match_planar_rotation(model_origin_points, ref_origin_points) {
				Some(A) => A,
				None => {
					println!("Unable to construct transformation matrix for model {}\n", model.name);
					continue
				}
			};

			let global_angle = global_transform.euler_angles();
			(ref_origin, global_angle, global_transform)

		} else {
			(Vector3::new(0f64, 0f64, 0f64), (0f64, 0f64, 0f64), Rotation3::identity())
		};

		#[cfg(feature = "debug")]
		let pre_transform_verts = get_avg_delta(&model_verts, &ref_verts);

		for i in 0..model_verts.len() {
			model_verts[i] = global_transform * model_verts[i] + ref_origin;
		}

		output += format!("[{}]\n", model.name).as_str();
		output += format!("Position={},{},{}\n", ref_origin.x, ref_origin.y, ref_origin.z).as_str();
		output += format!("Rotation={},{},{}\n\n", global_angle.0.to_degrees(), global_angle.1.to_degrees(), global_angle.2.to_degrees()).as_str();

		#[cfg(feature = "debug")]
		{
			output += format!("Origin: {:?}\n", ref_origin).as_str();
			output += format!("Blender Position={},{},{}\n", ref_origin.x, -ref_origin.z, ref_origin.y).as_str();
			output += format!("Blender Rotation={},{},{}\n", global_angle.0.to_degrees(), -global_angle.2.to_degrees(), global_angle.1.to_degrees()).as_str();
			output += format!("Average point delta: {}\n", get_avg_delta(&model_verts, &ref_verts)).as_str();
			output += format!("Average point delta pre-transform: {}\n\n", pre_transform_verts).as_str();
			println!("{}", output);
			output = String::new();
		}
	}

	#[cfg(not(feature = "debug"))]
	{
		let mut out = match File::create("output.txt") {
			Ok(f) => f,
			Err(e) => {
				println!("Failed to open output file: {}\nWaiting for 30 seconds, copy results now.", e);
				sleep(Duration::from_secs(30));
				return;
			}
		};

		match out.write_all(output.as_bytes()) {
			Ok(_) => {},
			Err(e) => {
				println!("Failed to write to output file: {}\nWaiting for 30 seconds, copy results now.", e);
				sleep(Duration::from_secs(30));
				return;
			}
		}

		println!("Outputted result to file \"output.txt\", this window will remain open for thirty seconds for you to copy the results manually.");
		sleep(Duration::from_secs(30));
	}
}


fn get_avg_point(verts: &Vec<Vector3<f64>>) -> Vector3<f64> {
	let mut out = verts.iter().sum::<Vector3<f64>>();
	out /= verts.len() as f64;
	out
}


fn get_point_dists(verts: &Vec<Vector3<f64>>, center: &Vector3<f64>) -> Vec<PointDist> {
	let mut out = Vec::with_capacity(verts.len());
	for vert in verts {
		out.push(PointDist { point: vert - center, dist: (center - vert).norm_squared()})
	}
	out.sort_by(|a, b | a.dist.total_cmp(&b.dist));
	out
}


fn get_avg_delta(verts: &Vec<Vector3<f64>>, ref_verts: &Vec<Vector3<f64>>) -> f64 {
	let model_pds = get_point_dists(verts, &Vector3::zeros());
	let ref_pds = get_point_dists(ref_verts, &Vector3::zeros());
	let len = model_pds.len() as f64;
	let mut out = 0f64;
	for (vert, ref_vert) in model_pds.iter().zip(&ref_pds) {
		out += vert.point.metric_distance(&ref_vert.point);
	}
	out /= len;
	out
}


fn match_planar_rotation(mut model_points: Vec<PointDist>, mut ref_points: Vec<PointDist>) -> Option<Rotation3<f64>> {

	let first_model = find_nonconflicting_point(&mut model_points)?;

	let first_ref = find_nonconflicting_point(&mut ref_points)?;

	let (model_cross, ref_cross) = loop {
		let second_model = find_nonconflicting_point(&mut model_points)?;
		let second_ref = find_nonconflicting_point(&mut ref_points)?;

		let model_cross = first_model.point.cross(&second_model.point);
		let ref_cross = first_ref.point.cross(&second_ref.point);
		if model_cross.norm_squared() > FLOATING_TOLERANCE { break (model_cross, ref_cross) }
	};

	let orientation = Rotation3::rotation_between(&model_cross, &ref_cross)?;
	let rotated_first_model: Vector3<f64> = orientation * first_model.point;
	let roll = Rotation3::rotation_between(&rotated_first_model, &first_ref.point)?;

	Some(roll * orientation)
}


fn find_nonconflicting_point(points: &mut Vec<PointDist>) -> Option<PointDist> {
	let mut out = points.pop()?;
	loop {
		let mut conflicting = false;
		loop {
			if (out.dist - points.last()?.dist).abs() < FLOATING_TOLERANCE {
				out = points.pop()?;
				conflicting = true;
			} else {
				break;
			}
		}
		if !conflicting {
			return Some(out);
		}
	}
}