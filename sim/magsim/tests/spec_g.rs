//! Device spec section G — gear physics acceptance tests (see tests/device_spec.md).
//! Written BEFORE the implementation (TDD): these must fail until magsim is real.

use magsim::{fit_sine_amplitude, harmonic, GearAssembly, GearParams};

fn design() -> GearParams {
    GearParams::default().validated().expect("default params must validate")
}

/// G0 (physics invariant): Newton's third law — member torques sum to ~zero.
/// If this fails the model is wrong everywhere else too.
#[test]
fn g0_newton_balance() {
    let p = design();
    let mut a = GearAssembly::build(&p);
    let t = a.torques(0.10, 0.02);
    assert!(
        t.imbalance().abs() < 0.02 * t.magnitude_max().max(1e-9),
        "torque imbalance {} vs max member {}",
        t.imbalance(),
        t.magnitude_max()
    );
}

/// G1: pole-pair algebra — ratio 4.25:1 within 0.1%, n_s rule, 3-port constraint.
#[test]
fn g1_ratio_correctness() {
    let p = design();
    assert_eq!(p.n_mod(), p.p_low + p.p_high, "modulator rule n_s = p1 + p3");
    let r = harmonic::ratio_fixed_modulator(&p);
    assert!(
        (r.abs() - 4.25).abs() / 4.25 < 0.001,
        "fixed-modulator ratio {r} != 4.25"
    );
    // 3-port constraint reduces to the two locked cases:
    let w_in = harmonic::omega_inner(&p, 1.0, 0.0);
    assert!((w_in - r).abs() < 1e-12);
    let w_in2 = harmonic::omega_inner(&p, 0.0, 1.0);
    assert!((w_in2 - harmonic::ratio_fixed_outer(&p)).abs() < 1e-12);
    // torque shares are consistent (lossless power balance at locked modulator)
    let (ti, tm, to) = harmonic::torque_shares_from_outer(&p, 70.0);
    assert!((ti + tm + to).abs() < 1e-9, "shares must balance");
    assert!((ti * p.p_high as f64 - to * p.p_low as f64).abs() < 1e-9);
}

/// G2a: capacity — crank-side pull-out ≥ 70 N·m for the (optimized) design.
#[test]
fn g2a_pullout_capacity() {
    let mut a = GearAssembly::build(&design());
    let po = a.pullout_crank();
    assert!(
        po.t_po >= 70.0,
        "pull-out {:.1} N·m < 70 N·m — design not yet strong enough",
        po.t_po
    );
}

/// G2b: two-model agreement — dipole-cloud vs harmonic estimate within 25%.
/// (The tighter 15% iron-free cross-check vs magpylib lives in validation/.)
#[test]
fn g2b_two_model_agreement() {
    let p = design();
    let mut a = GearAssembly::build(&p);
    let numeric = a.pullout_crank().t_po;
    let analytic = harmonic::analytic_pullout(&p);
    let rel = (numeric - analytic).abs() / numeric.max(1e-9);
    assert!(
        rel < 0.25,
        "dipole {numeric:.1} vs harmonic {analytic:.1} N·m differ {:.0}%",
        rel * 100.0
    );
}

/// G3: the load-angle characteristic is sinusoidal: T ≈ T_po·sin(p_high·δ), R² ≥ 0.98.
/// The entire slip-dynamics model (ridesim) rests on this shape.
#[test]
fn g3_sine_load_angle() {
    let p = design();
    let mut a = GearAssembly::build(&p);
    let curve = a.torque_curve_crank(48);
    let (amp, r2) = fit_sine_amplitude(&curve, p.p_high as f64);
    assert!(r2 >= 0.98, "R² = {r2:.3} < 0.98 — torque curve is not sin(pδ)");
    assert!(amp.abs() > 0.0, "zero amplitude");
    GearAssembly::write_curve_csv("../../results/torque_curve_crank.csv", &curve).ok();
}

/// G4: cogging < 2% of pull-out (pole combination + fill design rule works).
#[test]
fn g4_cogging_small() {
    let p = design();
    let mut a = GearAssembly::build(&p);
    let po = a.pullout_crank().t_po;
    let cog = a.cogging_curve(60);
    let mean = cog.iter().map(|(_, t)| t).sum::<f64>() / cog.len() as f64;
    let ripple = cog
        .iter()
        .map(|(_, t)| (t - mean).abs())
        .fold(0.0f64, f64::max);
    assert!(
        ripple < 0.02 * po,
        "cogging ripple {ripple:.2} N·m ≥ 2% of pull-out {po:.1}"
    );
}

/// S2 input: radial force on displaced inner rotor is finite, destabilizing
/// (negative stiffness, as physics demands), and exported for carrier design.
#[test]
fn s2_radial_stiffness_export() {
    let mut a = GearAssembly::build(&design());
    let f1 = a.radial_force_inner(0.0002);
    let f2 = a.radial_force_inner(0.0004);
    // magnetic radial pull grows with displacement (unstable direction)
    assert!(f2.x.abs() > f1.x.abs(), "radial force should grow with offset");
    let k = (f2.x - f1.x) / 0.0002; // N/m, expected positive (destabilizing) along +x
    std::fs::write(
        "../../results/radial_stiffness.txt",
        format!("k_radial_unstable_N_per_m,{k}\n"),
    )
    .ok();
}
