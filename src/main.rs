use std::env::args;
use std::env::vars;
use std::path::PathBuf;
use tobj::{LoadOptions, Model};


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

		let model_avg = get_avg_points(model);
		let ref_avg = get_avg_points(ref_model);
		let delta = [
			clamp_floating_error(ref_avg[0] - model_avg[0]),
			clamp_floating_error(ref_avg[1] - model_avg[1]),
			clamp_floating_error(model_avg[2] - model_avg[2])
		];

		if delta.iter().sum::<f64>() == 0f64 {
			println!("Model \"{}\" already aligned", model.name);
		} else {
			println!("Model \"{}\" delta position: (Δx = {}, Δy = {}, Δz = {})", model.name, delta[0], delta[2], delta[1]);
		}
	}
}


fn get_avg_points(model: &Model) -> [f64; 3] {
	let mut out = [0f64; 3];
	let len = (model.mesh.positions.len() / 3) as f64;
	for i in (0..model.mesh.positions.len()).step_by(3) {
		out[0] += model.mesh.positions[i] as f64 / len;
		out[1] += model.mesh.positions[i + 1] as f64 / len;
		out[2] += model.mesh.positions[i + 2] as f64 / len;
	}
	out
}


fn clamp_floating_error(val: f64) -> f64 {
	if -0.000_000_000_000_000_1 <= val && val <= 0.000_000_000_000_000_1 { return 0f64 }
	val
}