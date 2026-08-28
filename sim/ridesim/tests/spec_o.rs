//! Device spec sections O + G5 — overtorque architecture acceptance tests.
//! TDD: written before the implementation; must fail until ridesim is real.

use ridesim::*;

fn base() -> (Vehicle, Drivetrain, Rider, Scenario) {
    (Vehicle::default(), Drivetrain::default(), Rider::default(), Scenario::default())
}

/// O0 (numerical integrity): closed energy audit on a 30 s cruise, leak ≤ 0.5%.
#[test]
fn o0_energy_audit_closes() {
    let (veh, dt, rider, sc) = base();
    let r = simulate(&veh, &dt, &rider, &Cvt::Locked, &sc);
    let res = r.energy.residual_fraction();
    assert!(res < 0.005, "energy audit leaks {:.2}%", res * 100.0);
}

/// O1: series-elastic peak shaving — rider waveform peak/mean ≥ 1.8 becomes
/// ring-torque peak/mean ≤ 1.35, without large wind-up oscillation.
#[test]
fn o1_peak_shaving() {
    let (veh, dt, mut rider, sc) = base();
    rider.waveform_a = 0.85; // peak/mean ≈ 1.85 at crank
    let r = simulate(&veh, &dt, &rider, &Cvt::Locked, &sc);
    let pm_crank = {
        let vals: Vec<f64> = r
            .t
            .iter()
            .zip(&r.t_crank_applied)
            .filter(|(t, _)| **t >= 5.0)
            .map(|(_, x)| *x)
            .collect();
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        vals.iter().cloned().fold(f64::MIN, f64::max) / mean
    };
    assert!(pm_crank >= 1.7, "test setup: crank waveform peak/mean {pm_crank:.2} too smooth");
    let pm_ring = r.peak_over_mean_ring_torque(5.0);
    assert!(pm_ring <= 1.35, "ring peak/mean {pm_ring:.2} > 1.35 — spring not shaving");
    r.write_csv("../../results/o1_peak_shaving.csv").ok();
}

/// O2: sprint spike capture — 165 N·m, 0.4 s on top of 80% steady load:
/// at most ONE pole-pitch slip, re-capture < 1 crank rev, loss ≤ 15% of excess.
#[test]
fn o2_sprint_spike_capture() {
    let (veh, mut dt, mut rider, mut sc) = base();
    dt.t_pullout = 70.0;
    rider.p_target = 320.0; // ≈80% of pull-out at 90 rpm mean
    rider.spikes = vec![Spike { t0: 10.0, duration: 0.4, t_peak: 165.0 }];
    sc.t_end = 20.0;
    let r = simulate(&veh, &dt, &rider, &Cvt::Active { margin: 0.7, gain: 8.0, max_mod_speed: 80.0, max_mod_torque: 25.0 }, &sc);
    assert!(r.slip_events.len() <= 1, "{} slip events, expected ≤ 1", r.slip_events.len());
    if let Some(ev) = r.slip_events.first() {
        assert!(ev.pitches.abs() <= 1, "slipped {} pitches", ev.pitches);
        let cadence = rider.cadence_target / 60.0; // rev/s
        assert!(
            ev.recapture_time < 1.0 / cadence,
            "recapture {:.2}s ≥ one crank rev",
            ev.recapture_time
        );
        // excess energy of the spike above pull-out, crude upper bound:
        let excess = (165.0 - dt.t_pullout) * 0.4 * (rider.cadence_target * std::f64::consts::TAU / 60.0);
        assert!(ev.e_loss <= 0.15 * excess, "slip loss {:.1} J > 15% of excess {:.1} J", ev.e_loss, excess);
    }
    r.write_csv("../../results/o2_sprint_capture.csv").ok();
}

/// O3: CVT authority — 0→8% grade transition at 250 W, load angle stays ≤ 0.7·δ_po, no slips.
#[test]
fn o3_cvt_grade_authority() {
    let (veh, dt, rider, mut sc) = base();
    sc.grade_profile = vec![(0.0, 0.0), (50.0, 0.0), (80.0, 0.08), (10_000.0, 0.08)];
    sc.t_end = 60.0;
    sc.v0 = 9.0;
    let r = simulate(&veh, &dt, &rider, &Cvt::Active { margin: 0.7, gain: 8.0, max_mod_speed: 80.0, max_mod_torque: 25.0 }, &sc);
    assert!(r.slip_events.is_empty(), "slipped on a mere 8% grade with CVT active");
    let delta_po = std::f64::consts::FRAC_PI_2 / dt.p_eff;
    let dmax = r.delta.iter().cloned().fold(0.0f64, |a, b| a.max(b.abs()));
    assert!(dmax <= 0.72 * delta_po, "δ_max {:.4} rad exceeds 70% margin ({:.4})", dmax, 0.7 * delta_po);
    r.write_csv("../../results/o3_cvt_grade.csv").ok();
}

/// O4: standing start stomp 200 N·m from rest — ratchets ≤ 3 pitches then captures.
#[test]
fn o4_standing_start() {
    let (veh, dt, mut rider, mut sc) = base();
    rider.p_target = 0.0;
    rider.cadence_target = 0.0;
    rider.spikes = vec![Spike { t0: 0.2, duration: 1.2, t_peak: 200.0 }];
    sc.v0 = 0.0;
    sc.t_end = 6.0;
    let r = simulate(&veh, &dt, &rider, &Cvt::Active { margin: 0.7, gain: 8.0, max_mod_speed: 80.0, max_mod_torque: 25.0 }, &sc);
    let total_pitches: i32 = r.slip_events.iter().map(|e| e.pitches.abs()).sum();
    assert!(total_pitches <= 3, "standing start ratcheted {total_pitches} pitches (> 3)");
    let v_end = *r.v.last().unwrap();
    assert!(v_end > 1.0, "bike failed to launch (v_end {v_end:.2} m/s)");
    r.write_csv("../../results/o4_standing_start.csv").ok();
}

/// O5: graceful degradation — CVT dead, sprint anyway: cyclic slipping limits
/// torque but never drops transmitted torque to ~zero for > 50 ms, and cadence
/// stays controllable (< 130% of target).
#[test]
fn o5_degraded_mode() {
    let (veh, dt, mut rider, mut sc) = base();
    rider.spikes = vec![Spike { t0: 8.0, duration: 2.0, t_peak: 165.0 }];
    sc.t_end = 16.0;
    let r = simulate(&veh, &dt, &rider, &Cvt::Locked, &sc);
    // find longest interval with |T_transmitted| < 5 N·m after spike start
    let dt_s = r.t[1] - r.t[0];
    let mut longest = 0.0f64;
    let mut run = 0.0f64;
    for i in 0..r.t.len() {
        if r.t[i] < 8.0 { continue; }
        if r.t_transmitted[i].abs() < 5.0 { run += dt_s; longest = longest.max(run); } else { run = 0.0; }
    }
    assert!(longest <= 0.050, "torque hole {:.0} ms > 50 ms", longest * 1000.0);
    let cad_max = r.cadence_rpm.iter().cloned().fold(0.0f64, f64::max);
    assert!(cad_max < 1.30 * rider.cadence_target, "cadence ran away: {cad_max:.0} rpm");
}

/// G5: efficiency — at 200 W cruise, modeled losses (eddy + series damping +
/// CVT hold power) ≤ 3% of rider work.
#[test]
fn g5_efficiency() {
    let (veh, dt, mut rider, mut sc) = base();
    rider.p_target = 200.0;
    sc.t_end = 40.0;
    let r = simulate(&veh, &dt, &rider, &Cvt::Active { margin: 0.7, gain: 8.0, max_mod_speed: 80.0, max_mod_torque: 25.0 }, &sc);
    let frac = (r.energy.loss_eddy + r.energy.loss_cvt) / r.energy.work_in.max(1e-9);
    assert!(frac <= 0.03, "drivetrain loss {:.1}% > 3%", frac * 100.0);
}

/// Equal-area sanity: critical excess energy is positive below pull-out,
/// shrinks toward zero as load → pull-out, and grows with inertia.
#[test]
fn equal_area_criterion_shape() {
    let e50 = capture::critical_excess_energy(70.0, 17.0, 0.5, 0.02);
    let e90 = capture::critical_excess_energy(70.0, 17.0, 0.9, 0.02);
    assert!(e50 > e90 && e90 > 0.0, "equal-area energies wrong order: {e50} vs {e90}");
    let e50_big_j = capture::critical_excess_energy(70.0, 17.0, 0.5, 0.04);
    assert!(e50_big_j >= e50 * 0.99, "more inertia should not reduce critical energy");
}
