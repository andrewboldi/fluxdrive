//! # magsim — magnetostatics for the FluxDrive coaxial magnetic gear
//!
//! Physics from scratch, two independent models:
//!
//! 1. **Dipole-cloud numeric model** (`assembly`, `solver`): every magnet is
//!    subdivided into a cloud of point dipoles (uniform magnetization → moment
//!    m = M·V_cell, M = Br/μ0). Ferromagnetic modulator segments are clouds of
//!    *induced* dipoles solved to self-consistency by damped fixed-point
//!    iteration: M_iron = min(χ_eff·|B|/μ0, M_sat)·B̂. Forces/torques via the
//!    exact dipole–dipole interaction. This captures flux modulation, pull-out,
//!    cogging, and radial forces with no fitted constants.
//! 2. **Harmonic/analytic model** (`harmonic`): pole-pair algebra (ratios, the
//!    3-port speed constraint), torque shares, and a permeance-based pull-out
//!    estimate from the literature — the independent cross-check demanded by
//!    device spec G2/G3.
//!
//! ## Conventions
//! - SI units throughout: meters, Tesla, A·m² (dipole moment), N·m (torque).
//! - Gear axis = +z. Angles in radians. Torques are the z-component about the axis.
//! - "Inner rotor" carries `p_low` pole-pairs (high-speed, wheel side).
//!   "Outer ring" carries `p_high` pole-pairs (low-speed, crank side).
//!   Modulator has `n_mod = p_low + p_high` iron segments (frame/CVT port).
//! - Newton's third law: torques on (inner + modulator + outer) must sum to ~0.
//!   This is enforced by test `g0_newton_balance` — it is the model's smoke alarm.

pub mod vec3;
pub use vec3::Vec3;

pub const MU0: f64 = 1.256_637_062e-6;

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

/// Full geometric + material description of the gear. Radii are in meters and
/// must satisfy: r_inner_bore < r_inner_mag_in < r_inner_mag_out < (+gap)
/// r_mod_in < r_mod_out < (+gap) r_outer_mag_in < r_outer_mag_out.
#[derive(Clone, Debug)]
pub struct GearParams {
    /// pole-pairs on the inner (high-speed, wheel-side) rotor, e.g. 4
    pub p_low: u32,
    /// pole-pairs on the outer (low-speed, crank-side) ring, e.g. 17
    pub p_high: u32,
    /// inner rotor magnet radial extent [m]
    pub r_inner_mag_in: f64,
    pub r_inner_mag_out: f64,
    /// air gaps (inner–modulator and modulator–outer) [m]
    pub air_gap: f64,
    /// modulator segment radial thickness [m]
    pub t_mod: f64,
    /// outer ring magnet radial thickness [m]
    pub t_outer_mag: f64,
    /// active axial length [m]
    pub axial_len: f64,
    /// magnet remanence Br [T] (sintered NdFeB N42 ≈ 1.32)
    pub br: f64,
    /// fraction of each pole arc actually covered by magnet (pocket walls) 0..1
    pub arc_fill: f64,
    /// fraction of modulator annulus covered by iron segments (typ. ~0.5)
    pub mod_fill: f64,
    /// iron saturation magnetization M_sat [A/m] (mild steel ≈ 1.7e6)
    pub m_sat: f64,
    /// effective susceptibility for the induced-iron fixed point (large, e.g. 1000)
    pub chi_eff: f64,
    /// dipole subdivision per magnet/iron segment: (radial, azimuthal, axial)
    pub subdiv: (u32, u32, u32),
}

impl GearParams {
    pub fn n_mod(&self) -> u32 {
        self.p_low + self.p_high
    }
    /// Kinematic ratio magnitude with modulator fixed (crank ring : wheel rotor).
    pub fn nominal_ratio(&self) -> f64 {
        self.p_high as f64 / self.p_low as f64
    }
    /// Check geometric consistency; returns self or a description of the violation.
    pub fn validated(self) -> Result<GearParams, String> {
        let g = &self;
        let seq = [
            g.r_inner_mag_in,
            g.r_inner_mag_out,
            g.r_inner_mag_out + g.air_gap,
            g.r_inner_mag_out + g.air_gap + g.t_mod,
            g.r_inner_mag_out + 2.0 * g.air_gap + g.t_mod,
            g.r_inner_mag_out + 2.0 * g.air_gap + g.t_mod + g.t_outer_mag,
        ];
        if seq.windows(2).any(|w| w[1] <= w[0]) {
            return Err(format!("inconsistent radii: {seq:?}"));
        }
        if g.p_low == 0 || g.p_high <= g.p_low {
            return Err("need 0 < p_low < p_high".into());
        }
        if !(0.0..=1.0).contains(&g.arc_fill) || !(0.0..=1.0).contains(&g.mod_fill) {
            return Err("fills must be in [0,1]".into());
        }
        Ok(self)
    }
    pub fn r_mod_in(&self) -> f64 {
        self.r_inner_mag_out + self.air_gap
    }
    pub fn r_mod_out(&self) -> f64 {
        self.r_mod_in() + self.t_mod
    }
    pub fn r_outer_mag_in(&self) -> f64 {
        self.r_mod_out() + self.air_gap
    }
    pub fn r_outer_mag_out(&self) -> f64 {
        self.r_outer_mag_in() + self.t_outer_mag
    }
}

impl Default for GearParams {
    /// v1 design guess (to be optimized): 4/17 with 21 modulator segments,
    /// ~190 mm outer active diameter, 60 mm stack — sized for ≥70 N·m pull-out.
    fn default() -> Self {
        GearParams {
            p_low: 4,
            p_high: 17,
            r_inner_mag_in: 0.052,
            r_inner_mag_out: 0.064,
            air_gap: 0.0015,
            t_mod: 0.010,
            t_outer_mag: 0.012,
            axial_len: 0.060,
            br: 1.32,
            arc_fill: 0.90,
            mod_fill: 0.50,
            m_sat: 1.7e6,
            chi_eff: 1000.0,
            subdiv: (2, 3, 3),
        }
    }
}

// ---------------------------------------------------------------------------
// Assembly + solver (dipole-cloud numeric model)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Member {
    Inner,
    Modulator,
    Outer,
}

#[derive(Clone, Copy, Debug)]
pub struct MemberTorques {
    pub inner: f64,
    pub modulator: f64,
    pub outer: f64,
}

impl MemberTorques {
    pub fn magnitude_max(&self) -> f64 {
        self.inner.abs().max(self.modulator.abs()).max(self.outer.abs())
    }
    pub fn imbalance(&self) -> f64 {
        self.inner + self.modulator + self.outer
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Pullout {
    /// peak transmittable torque on the crank-side (outer) ring [N·m]
    pub t_po: f64,
    /// crank-side mechanical load angle at pull-out [rad]
    pub delta_po: f64,
}

pub struct GearAssembly {
    // implementation-private dipole clouds etc.
    _private: (),
}

impl GearAssembly {
    /// Build dipole clouds for all three members from validated params.
    pub fn build(_p: &GearParams) -> Self {
        todo!("dipole cloud construction")
    }

    /// Rotate inner rotor to `angle_in` and outer ring to `angle_out`
    /// (modulator fixed at 0), solve induced iron magnetization to
    /// self-consistency, and return axial torques on all three members.
    pub fn torques(&mut self, _angle_in: f64, _angle_out: f64) -> MemberTorques {
        todo!("induced-iron fixed point + dipole torque summation")
    }

    /// Static transmitted-torque curve vs crank-side load angle. The outer ring
    /// is displaced by delta ∈ [0, 2π/p_high) from the aligned equilibrium with
    /// the inner rotor held; returns (delta, torque_on_outer).
    pub fn torque_curve_crank(&mut self, n_pts: usize) -> Vec<(f64, f64)> {
        let _ = n_pts;
        todo!()
    }

    /// Maximum of the torque curve (crank side).
    pub fn pullout_crank(&mut self) -> Pullout {
        todo!()
    }

    /// Torque ripple during synchronized no-load rotation (cogging).
    /// Both rotors advance kinematically consistently (θ_out = θ, θ_in =
    /// -θ·p_high/p_low) at zero mean load angle; returns (θ_crank, T_outer).
    pub fn cogging_curve(&mut self, n_pts: usize) -> Vec<(f64, f64)> {
        let _ = n_pts;
        todo!()
    }

    /// Net radial force on the inner rotor when displaced by `dx` [m] from
    /// center (for maglev/carrier stiffness studies, spec S2/B2).
    pub fn radial_force_inner(&mut self, _dx: f64) -> Vec3 {
        todo!()
    }

    /// Write a torque curve as CSV (delta,torque) for plotting/validation.
    pub fn write_curve_csv(path: &str, curve: &[(f64, f64)]) -> std::io::Result<()> {
        let mut s = String::from("x,torque\n");
        for (x, t) in curve {
            s.push_str(&format!("{x},{t}\n"));
        }
        std::fs::write(path, s)
    }
}

// ---------------------------------------------------------------------------
// Harmonic / analytic model (independent cross-check)
// ---------------------------------------------------------------------------

pub mod harmonic {
    use super::GearParams;

    /// Signed ratio ω_inner/ω_outer with modulator fixed: −p_high/p_low.
    pub fn ratio_fixed_modulator(p: &GearParams) -> f64 {
        -(p.p_high as f64) / (p.p_low as f64)
    }

    /// Signed ratio ω_inner/ω_mod with outer ring fixed: n_mod/p_low.
    pub fn ratio_fixed_outer(p: &GearParams) -> f64 {
        p.n_mod() as f64 / p.p_low as f64
    }

    /// 3-port kinematic constraint: p_low·ω_inner + p_high·ω_outer = n_mod·ω_mod.
    /// Given two speeds, return the third (here: ω_inner from ω_outer, ω_mod).
    pub fn omega_inner(p: &GearParams, omega_outer: f64, omega_mod: f64) -> f64 {
        (p.n_mod() as f64 * omega_mod - p.p_high as f64 * omega_outer) / p.p_low as f64
    }

    /// Steady-state torque shares |T_inner| : |T_mod| : |T_outer| = p_low : n_mod : p_high
    /// (lossless). Returns torques on (inner, mod, outer) given torque on outer.
    pub fn torque_shares_from_outer(p: &GearParams, t_outer: f64) -> (f64, f64, f64) {
        let t_inner = t_outer * p.p_low as f64 / p.p_high as f64;
        (t_inner, -(t_inner + t_outer), t_outer)
    }

    /// Permeance/harmonic estimate of crank-side pull-out torque [N·m].
    /// Implementers: use a documented literature formula (e.g. Lubin-style
    /// subdomain leading harmonic, or the standard T ∝ Br² scaling calibrated
    /// per the research notes) — cite the formula in a doc comment.
    pub fn analytic_pullout(_p: &GearParams) -> f64 {
        todo!("permeance-model pull-out estimate")
    }
}

// ---------------------------------------------------------------------------
// Magnet-magnet coaxial coupling (series-elastic spring; magpylib-validatable)
// ---------------------------------------------------------------------------

pub mod coupling {
    /// A contactless torsion spring: two concentric rings of alternating-pole
    /// magnets (no iron!), used between crank and gear input for peak shaving.
    /// Iron-free ⇒ directly cross-validatable against magpylib (spec G2 route).
    #[derive(Clone, Debug)]
    pub struct CouplingParams {
        /// pole pairs on each ring
        pub p: u32,
        pub r_in_mag_out: f64,
        pub t_in_mag: f64,
        pub air_gap: f64,
        pub t_out_mag: f64,
        pub axial_len: f64,
        pub br: f64,
        pub arc_fill: f64,
        pub subdiv: (u32, u32, u32),
    }

    impl Default for CouplingParams {
        fn default() -> Self {
            CouplingParams {
                p: 6,
                r_in_mag_out: 0.045,
                t_in_mag: 0.008,
                air_gap: 0.002,
                t_out_mag: 0.008,
                axial_len: 0.030,
                br: 1.32,
                arc_fill: 0.9,
                subdiv: (2, 3, 3),
            }
        }
    }

    pub struct CoaxialCoupling {
        _private: (),
    }

    impl CoaxialCoupling {
        pub fn build(_p: &CouplingParams) -> Self {
            todo!()
        }
        /// Torque on outer ring vs relative angle delta [rad].
        pub fn torque(&self, _delta: f64) -> f64 {
            todo!()
        }
        /// dT/dδ at δ=0 [N·m/rad] — the series-elastic stiffness.
        pub fn stiffness_at_zero(&self) -> f64 {
            todo!()
        }
        /// Serialize the exact magnet layout (positions, dims, magnetizations)
        /// as JSON for the Python/magpylib cross-check (validation/).
        pub fn export_case_json(&self, _path: &str) -> std::io::Result<()> {
            todo!()
        }
    }
}

// ---------------------------------------------------------------------------
// Shared numeric utilities available to tests
// ---------------------------------------------------------------------------

/// Least-squares fit of y ≈ A·sin(k·x) for known k; returns (A, r_squared).
pub fn fit_sine_amplitude(curve: &[(f64, f64)], k: f64) -> (f64, f64) {
    let (mut num, mut den) = (0.0, 0.0);
    for (x, y) in curve {
        let s = (k * x).sin();
        num += y * s;
        den += s * s;
    }
    let a = if den > 0.0 { num / den } else { 0.0 };
    let mean = curve.iter().map(|(_, y)| y).sum::<f64>() / curve.len() as f64;
    let (mut ss_res, mut ss_tot) = (0.0, 0.0);
    for (x, y) in curve {
        let f = a * (k * x).sin();
        ss_res += (y - f) * (y - f);
        ss_tot += (y - mean) * (y - mean);
    }
    let r2 = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 1.0 };
    (a, r2)
}
