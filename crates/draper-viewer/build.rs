// Build script for draper-viewer: inject git revision hash at compile time.
// Works for both native and wasm32 targets.

use std::process::Command;

fn main() {
    let git_hash = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout).ok().map(|s| s.trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "unknown".to_string());

    let git_dirty = Command::new("git")
        .args(["diff", "--quiet"])
        .output()
        .ok()
        .map(|o| !o.status.success())
        .unwrap_or(false);

    let revision = if git_dirty {
        format!("{}+", git_hash)
    } else {
        git_hash
    };

    println!("cargo:rustc-env=DRAPER_GIT_HASH={}", revision);
    // Re-run build script if git HEAD changes
    println!("cargo:rerun-if-changed=.git/HEAD");
}
