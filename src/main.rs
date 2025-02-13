use std::env::args;
use std::env::vars;
use std::path::PathBuf;
use nalgebra::{Matrix3, Rotation3, Vector3};
use tobj::LoadOptions;


const BLENDER_TRANSFORM: Matrix3<f64> = Matrix3::new(1f64, 0f64, 0f64, 0f64, 0f64, 1f64, 0f64, -1f64, 0f64);


struct PointDist {
	point: Vector3<f64>,
	dist: f64,
}

impl PartialEq for PointDist {
	fn eq(&self, other: &Self) -> bool {
		(self.dist - other.dist).abs() < 0.000_000_1f64
	}
}


fn main() {
	let args = args().collect::<Vec<String>>();
	#[cfg(not(feature = "debug"))]
	let (obj, reference) = if args.len() < 3 {
		let obj = rfd::FileDialog::new()
			.add_filter("Wavefront .obj", &[".obj"])
			.set_title("Un-rigged obj")
			.pick_file();
		if obj.is_none() { return }

		let reference = rfd::FileDialog::new()
			.add_filter("Wavefront .obj", &[".obj"])
			.set_title("Rigged (reference) obj")
			.pick_file();
		if reference.is_none() { return }

		(obj.unwrap(), reference.unwrap())
	} else {
		(PathBuf::from(args[1]), PathBuf::from(args[2]))
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

		let model_center = get_avg_points(&model_verts);
		let ref_center = get_avg_points(&ref_verts);

		let model_local_point_dists = get_point_dists(&model_verts, &model_center);
		let ref_local_point_dists = get_point_dists(&ref_verts, &ref_center);

		if model_local_point_dists[0] != ref_local_point_dists[0] { println!("Model \"{}\" is scaled differently than its reference counterpart or has different vertices | {:?}, {:?}", model.name, model_local_point_dists[0].dist, ref_local_point_dists[0].dist); }

		let local_rot = Rotation3::rotation_between(&(model_local_point_dists[0].point - model_center), &(ref_local_point_dists[0].point - ref_center)).unwrap(); // TODO: Gracefully handle error
		let ref_origin: Vector3<f64> = ref_center - local_rot * model_center;

		let model_origin_point_dists = get_point_dists(&model_verts, &Vector3::zeros());
		let ref_origin_point_dists = get_point_dists(&ref_verts, &ref_origin);

		let global_rot = Rotation3::rotation_between(&model_origin_point_dists[0].point, &(ref_origin_point_dists[0].point - ref_origin)).unwrap(); // TODO: Gracefully handle error

		let eulers = global_rot.euler_angles();

		for i in 0..model_verts.len() {
			model_verts[i] = global_rot * model_verts[i];
		}

		let delta = (ref_origin).map(|x| clamp_floating_error(x));

		println!("[{}]", model.name);
		println!("Position={},{},{}", delta[0], delta[1], delta[2]);
		println!("Rotation={},{},{}", eulers.0.to_degrees(), eulers.1.to_degrees(), eulers.2.to_degrees());
		#[cfg(feature = "blender")]
		{
			println!("blender local rot: {}, {}, {}", local_rot.euler_angles().0.to_degrees(), -local_rot.euler_angles().2.to_degrees(), local_rot.euler_angles().1.to_degrees());
			println!("blender origin: {:?}", BLENDER_TRANSFORM * ref_origin);
			println!("Blender Position={},{},{}", delta[0], -delta[2], delta[1]);
			println!("Blender Rotation={},{},{}", eulers.0.to_degrees(), -eulers.2.to_degrees(), eulers.1.to_degrees());
		}
		#[cfg(feature = "debug")]
		{
			println!("Average point delta: {}", get_avg_delta(&model_verts, &ref_verts));
		}
	}
}


fn get_avg_points(verts: &Vec<Vector3<f64>>) -> Vector3<f64> {
	let mut out = Vector3::zeros();
	let len = verts.len() as f64;
	for vert in verts {
		out += vert;
	}
	out /= len;
	out
}


fn clamp_floating_error(val: f64) -> f64 {
	if -0.000_000_000_000_1 <= val && val <= 0.000_000_000_000_1 { return 0f64 }
	val
}


fn get_point_dists(verts: &Vec<Vector3<f64>>, center: &Vector3<f64>) -> Vec<PointDist> {
	let mut out = Vec::with_capacity(verts.len());
	for vert in verts {
		out.push(PointDist { point: vert.clone(), dist: (center - vert).norm_squared()})
	}

	out.sort_by(|a, b | b.dist.total_cmp(&a.dist));
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