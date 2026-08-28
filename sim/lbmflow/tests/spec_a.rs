//! Device spec section A — CFD solver validation before any fairing claims.
//! TDD: these fail until lbmflow is implemented.
//! The heavy cylinder run is #[ignore] by default; run the full ladder with:
//!   cargo test -p lbmflow --release -- --include-ignored

use lbmflow::*;

/// A0: Taylor–Green vortex — effective viscosity within 2% of theory.
/// This is the solver's unit-physics test: if collision/streaming are wrong,
/// nothing downstream matters.
#[test]
fn a0_taylor_green_viscosity() {
    let (nu_fit, nu_theory) = taylor_green_viscosity(96, 0.8, 2000);
    let rel = (nu_fit - nu_theory).abs() / nu_theory;
    assert!(rel < 0.02, "viscosity off by {:.1}% (fit {nu_fit:.6} vs {nu_theory:.6})", rel * 100.0);
}

/// A1: cylinder at Re=100 — Cd_mean ≈ 1.33 ± 10%, Strouhal in [0.15, 0.20].
#[test]
#[ignore = "heavy: ~1e5 steps; run with --include-ignored --release"]
fn a1_cylinder_re100() {
    let b = cylinder_benchmark(100.0, 26);
    assert!(
        (b.cd_mean - 1.33).abs() / 1.33 < 0.10,
        "Cd_mean {:.3} not within 10% of 1.33",
        b.cd_mean
    );
    assert!(
        b.strouhal > 0.15 && b.strouhal < 0.20,
        "Strouhal {:.3} outside [0.15, 0.20]",
        b.strouhal
    );
}

/// A2 (placeholder until CAD provides profiles): best fairing beats the bare
/// gearbox pod section by ≥ 15% at matched Re. Enabled once profiles exist in
/// cad/profiles/*.csv.
#[test]
#[ignore = "enabled when cad/profiles exist"]
fn a2_fairing_gain() {
    let pod: Vec<(f64, f64)> = (0..64)
        .map(|i| {
            let a = i as f64 / 64.0 * std::f64::consts::TAU;
            (a.cos() * 0.5, a.sin() * 0.5)
        })
        .collect(); // circle = unfaired pod section
    let cd_pod = profile_drag(&pod, 1000.0, 40);
    let fairing: Vec<(f64, f64)> = (0..64)
        .map(|i| {
            let a = i as f64 / 64.0 * std::f64::consts::TAU;
            (a.cos() * 1.5, a.sin() * 0.5) // 3:1 ellipse stand-in until real profile
        })
        .collect();
    let cd_fair = profile_drag(&fairing, 1000.0, 40);
    assert!(
        cd_fair < 0.85 * cd_pod,
        "fairing Cd {cd_fair:.3} not ≥15% better than pod {cd_pod:.3}"
    );
}
