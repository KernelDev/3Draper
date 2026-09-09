// Verify extract_step_tolerance on a list of files (arg-provided, fallback
// to a default set). Reports the extracted uncertainty per file.
use draper_step::{parser::parse_step, extract_step_tolerance};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let files: Vec<String> = if args.is_empty() {
        vec![
            "test/as1-oc-214.stp".into(),
            "test/drill_top.stp".into(),
            "test/Zentralstaender.stp".into(),
            "test/compressor-13920_top.stp".into(),
            "test/SampleCube.step".into(),
            "test/8394-121_Spit-Fire.STEP".into(),
            "test/8500-02_Vulcan.STEP".into(),
        ]
    } else {
        args
    };
    for f in files {
        let content = match std::fs::read_to_string(&f) {
            Ok(c) => c,
            Err(e) => {
                println!("{f}: READ ERROR {e}");
                continue;
            }
        };
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => {
                println!("{f}: PARSE ERROR {e}");
                continue;
            }
        };
        let tol = extract_step_tolerance(&step);
        match tol {
            Some(t) => println!("{f}: uncertainty = {t:.3e}"),
            None => println!("{f}: uncertainty = NONE (model_scale fallback)"),
        }
    }
}
