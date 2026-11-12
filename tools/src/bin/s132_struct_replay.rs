//! s132 offline replay: load a CDT_FAIL dump (BOUNDARY/INTERIOR
//! sections) and run the STRUCTURAL lattice rescue on it directly —
//! instant iteration on the #27977 FAT-fold faces without a full
//! STEP conversion. Reports accept/reject + guard stats.
//! Usage: s132_struct_replay <dump.cdtfail.txt> [more dumps...]
//! Env: DRAPPER_LATTICE_DEBUG=1 DRAPPER_BAND_CHAINS=1 (diagnostics).
use draper_mesh::custom_cdt::structural_lattice_rescue_legacy;

fn parse_dump(path: &str) -> Option<(Vec<[f64; 2]>, Vec<[f64; 2]>)> {
    let text = std::fs::read_to_string(path).ok()?;
    let mut sec = "";
    let mut bnd: Vec<[f64; 2]> = Vec::new();
    let mut int: Vec<[f64; 2]> = Vec::new();
    for ln in text.lines() {
        match ln.trim() {
            "BOUNDARY" => sec = "b",
            "HOLES" => sec = "h",
            "INTERIOR" => sec = "i",
            "LEGACY" | "CDT" => sec = "",
            _ => {}
        }
        if let Some(rest) = ln.strip_prefix("HOLE ") {
            sec = "hole";
            let _ = rest;
            continue;
        }
        if ln.starts_with("n_boundary") {
            continue;
        }
        let p: Vec<f64> = ln
            .split_whitespace()
            .filter_map(|t| t.parse::<f64>().ok())
            .collect();
        match sec {
            "b" if p.len() == 2 => bnd.push([p[0], p[1]]),
            "i" if p.len() == 2 => int.push([p[0], p[1]]),
            _ => {}
        }
    }
    if bnd.len() < 3 || int.is_empty() {
        None
    } else {
        Some((bnd, int))
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: s132_struct_replay <dump.cdtfail.txt> [...]");
        std::process::exit(2);
    }
    let mut n_ok = 0;
    for path in &args {
        let name = path.rsplit('/').next().unwrap_or(path);
        let Some((bnd, int)) = parse_dump(path) else {
            println!("{}: PARSE FAIL", name);
            continue;
        };
        match structural_lattice_rescue_legacy(&bnd, &int) {
            Some(tris) => {
                // guard summary: points used, edge census
                let total = bnd.len() + int.len();
                let mut used = vec![false; total];
                for t in &tris {
                    for &v in t {
                        if (v as usize) < total {
                            used[v as usize] = true;
                        }
                    }
                }
                let unused = used.iter().filter(|&&u| !u).count();
                println!(
                    "{}: ACCEPT tris={} unused_pts={} (bnd={} int={})",
                    name,
                    tris.len(),
                    unused,
                    bnd.len(),
                    int.len()
                );
                n_ok += 1;
            }
            None => println!("{}: REJECT", name),
        }
    }
    println!("accepted {}/{}", n_ok, args.len());
}
