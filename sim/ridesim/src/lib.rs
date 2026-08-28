//! # ridesim — FluxDrive system dynamics
//!
//! Simulates the full torque path: rider legs → crank → **series-elastic
//! magnetic torsion spring** → gear input ring (p_high) → **flux-modulated
//! magnetic gear** (load-angle dynamics, can slip poles) → wheel → bike
//! longitudinal dynamics. The modulator is the CVT control port.
//!
//! ## The core physics: load-angle ("synchronous machine") dynamics
//! The magnetic gear transmits T(δ) = T_po·sin(p_eff·δ) where δ is the
//! mechanical load angle between the crank-side ring and the (ratio-scaled)
//! wheel-side rotor, p_eff = p_high. Exceed T_po and the gear *slips a pole
//! pitch* — exactly a synchronous machine losing step. Capture/re-capture is
//! governed by the **equal-area criterion**: a torque spike is survivable iff
//! the kinetic energy pumped into the relative coordinate before δ passes the
//! unstable equilibrium is absorbed by the decelerating area of T(δ).
//!
//! FluxDrive's OPSR stack, all modeled here:
//!  1. series-elastic spring (stiffness k_series from magsim coupling) shaves
//!     the 2/rev pedal peaks;
//!  2. CVT: modulator speed control reshapes demanded crank torque;
//!  3. phase-slip recovery: slip events ratchet exactly one pitch, energy is
//!     audited (rotor KE + spring), and re-capture is verified dynamically.
//!
//! ## Conventions
//! SI units; angles rad; speeds rad/s; energies J. Fixed-step RK4 integrator
//! (dt ≲ 1e-4 s — the electrical stiffness p_eff·T_po is high). Every sim
//! carries a running **energy audit**; leaks > 0.5% fail the spec.

pub const G: f64 = 9.806;

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub mass: f64,     // rider+bike [kg]
    pub r_wheel: f64,  // [m]
    pub crr: f64,      // rolling resistance
    pub cda: f64,      // [m²]
    pub rho: f64,      // air density
}

impl Default for Vehicle {
    fn default() -> Self {
        Vehicle { mass: 85.0, r_wheel: 0.335, crr: 0.005, cda: 0.32, rho: 1.225 }
    }
}

#[derive(Clone, Debug)]
pub struct Drivetrain {
    /// electrical pole multiplier of the load-angle characteristic (= p_high)
    pub p_eff: f64,
    /// crank-side pull-out torque of the gear [N·m] (from magsim)
    pub t_pullout: f64,
    /// kinematic ratio |ω_wheel/ω_crank| with modulator locked (= p_high/p_low)
    pub ratio: f64,
    /// series-elastic magnetic spring stiffness [N·m/rad] and loss [N·m·s/rad]
    pub k_series: f64,
    pub c_series: f64,
    /// inertias [kg·m²]: crank+pedals+rider-limb reflected; gear input ring;
    /// wheel-side rotor; wheel+tire
    pub j_crank: f64,
    pub j_ring: f64,
    pub j_rotor: f64,
    pub j_wheel: f64,
    /// intrinsic damping of the gear coupling (eddy) [N·m·s/rad on δ̇]
    pub c_gap: f64,
    /// speed-proportional drivetrain loss torque referred to crank [N·m·s/rad]
    pub eddy_k: f64,
    /// pole pairs on low/high side for the 3-port constraint
    pub p_low: f64,
    pub p_high: f64,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Drivetrain {
            p_eff: 17.0,
            t_pullout: 70.0,
            ratio: 4.25,
            k_series: 300.0,
            c_series: 0.8,
            j_crank: 0.45,
            j_ring: 0.020,
            j_rotor: 0.004,
            j_wheel: 0.14,
            c_gap: 0.05,
            eddy_k: 0.010,
            p_low: 4.0,
            p_high: 17.0,
        }
    }
}

/// Rider torque source: T(θ) = T_mean(t)·(1 + a·sin(2θ + φ)), clamped ≥ 0,
/// with closed-loop mean torque tracking a power/cadence target, plus optional
/// deterministic spike events (sprints, standing starts).
#[derive(Clone, Debug)]
pub struct Rider {
    pub p_target: f64,        // [W]
    pub cadence_target: f64,  // [rpm]
    pub waveform_a: f64,      // 2/rev amplitude (≈0.85 ⇒ peak/mean ≈ 1.85)
    pub phase: f64,
    pub spikes: Vec<Spike>,
}

#[derive(Clone, Copy, Debug)]
pub struct Spike {
    pub t0: f64,
    pub duration: f64,
    pub t_peak: f64, // crank torque plateau during the spike [N·m]
}

impl Default for Rider {
    fn default() -> Self {
        Rider { p_target: 250.0, cadence_target: 90.0, waveform_a: 0.85, phase: 0.0, spikes: vec![] }
    }
}

#[derive(Clone, Debug)]
pub enum Cvt {
    /// modulator rigidly held (pure fixed-ratio gear)
    Locked,
    /// active modulator speed control: regulate load angle to margin·δ_po
    Active { margin: f64, gain: f64, max_mod_speed: f64, max_mod_torque: f64 },
}

#[derive(Clone, Debug)]
pub struct Scenario {
    /// piecewise-linear grade vs distance [(x [m], grade)] (grade = tan θ)
    pub grade_profile: Vec<(f64, f64)>,
    pub v0: f64,     // initial speed [m/s]
    pub t_end: f64,  // [s]
    pub dt: f64,     // [s]
}

impl Default for Scenario {
    fn default() -> Self {
        Scenario { grade_profile: vec![(0.0, 0.0)], v0: 8.0, t_end: 30.0, dt: 5e-5 }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SlipEvent {
    pub t: f64,
    /// pole pitches slipped in this event (signed)
    pub pitches: i32,
    /// energy dissipated in the event [J]
    pub e_loss: f64,
    /// time from limit crossing to re-capture [s]
    pub recapture_time: f64,
}

#[derive(Clone, Debug, Default)]
pub struct EnergyAudit {
    pub work_in: f64,       // rider work [J]
    pub delta_ke: f64,      // Δ kinetic energy (all inertias + vehicle) [J]
    pub delta_pe: f64,      // Δ potential energy (climb) [J]
    pub loss_aero: f64,
    pub loss_rr: f64,
    pub loss_eddy: f64,     // gear + series damping + slip losses [J]
    pub loss_cvt: f64,      // modulator control electrical loss [J]
    pub spring_stored: f64, // energy in series spring at end [J]
    pub gap_stored: f64,    // energy in the magnetic gap "spring" at end [J]
}

impl EnergyAudit {
    /// |in − (ΔKE + ΔPE + losses + stored)| / work_in
    pub fn residual_fraction(&self) -> f64 {
        let out = self.delta_ke + self.delta_pe + self.loss_aero + self.loss_rr
            + self.loss_eddy + self.loss_cvt + self.spring_stored + self.gap_stored;
        (self.work_in - out).abs() / self.work_in.max(1e-9)
    }
}

#[derive(Clone, Debug, Default)]
pub struct SimResult {
    pub t: Vec<f64>,
    pub cadence_rpm: Vec<f64>,
    pub v: Vec<f64>,
    pub delta: Vec<f64>,          // load angle [rad]
    pub t_crank_applied: Vec<f64>, // rider torque at crank
    pub t_ring: Vec<f64>,          // torque entering gear after series spring
    pub t_transmitted: Vec<f64>,   // gear coupling torque (crank side)
    pub ratio_effective: Vec<f64>, // instantaneous ω_wheel/ω_crank
    pub slip_events: Vec<SlipEvent>,
    pub energy: EnergyAudit,
}

impl SimResult {
    pub fn write_csv(&self, path: &str) -> std::io::Result<()> {
        let mut s = String::from("t,cadence_rpm,v,delta,t_crank,t_ring,t_transmitted,ratio\n");
        for i in 0..self.t.len() {
            s.push_str(&format!(
                "{},{},{},{},{},{},{},{}\n",
                self.t[i], self.cadence_rpm[i], self.v[i], self.delta[i],
                self.t_crank_applied[i], self.t_ring[i], self.t_transmitted[i],
                self.ratio_effective[i]
            ));
        }
        std::fs::write(path, s)
    }
    pub fn peak_over_mean_ring_torque(&self, t_from: f64) -> f64 {
        let vals: Vec<f64> = self
            .t
            .iter()
            .zip(&self.t_ring)
            .filter(|(t, _)| **t >= t_from)
            .map(|(_, x)| *x)
            .collect();
        let mean = vals.iter().sum::<f64>() / vals.len().max(1) as f64;
        let peak = vals.iter().cloned().fold(f64::MIN, f64::max);
        peak / mean.max(1e-9)
    }
}

/// Run the full coupled simulation.
pub fn simulate(
    _veh: &Vehicle,
    _dt: &Drivetrain,
    _rider: &Rider,
    _cvt: &Cvt,
    _sc: &Scenario,
) -> SimResult {
    todo!("RK4 integration of the coupled crank/spring/gear/vehicle system")
}

/// Equal-area criterion analysis on the static curve: for operating load
/// torque fraction `load_frac` = T_load/T_po, return the maximum *excess
/// impulse energy* [J] a spike may inject into the relative coordinate without
/// losing synchronism (single-swing criterion, inertia `j_rel` about δ).
pub mod capture {
    pub fn critical_excess_energy(_t_po: f64, _p_eff: f64, _load_frac: f64, _j_rel: f64) -> f64 {
        todo!("equal-area integral between stable and unstable equilibria")
    }
}
