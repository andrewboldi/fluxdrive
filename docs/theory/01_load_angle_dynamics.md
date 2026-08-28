# FluxDrive Theory I — Load-Angle Dynamics, Equal-Area Capture, and the OPSR Architecture

*Derived from first principles; the contracts implemented by `magsim`/`ridesim` and cited by the paper.*

## 1. Torque transmission in a flux-modulated gear

Members: inner rotor with `p_l` pole-pairs (wheel side, high speed), outer ring with `p_h`
pole-pairs (crank side, low speed), modulator with `n_s = p_l + p_h` iron segments.

To leading space-harmonic order, the coupling energy depends only on the *electrical
alignment*

```
δe = p_h·θ_h + p_l·θ_l + const        (modulator fixed at 0)
U(δe) = −(T_po/p_h)·cos(δe)
```

Torques follow from T = −∂U/∂θ:

```
T_h = −T_po·sin(δe)                    (on crank ring)
T_l = −(p_l/p_h)·T_po·sin(δe)          (on wheel rotor)
```

so torque magnitudes are shared as `p_l : p_h`, and with the modulator as third port,
`|T_mod| : |T_h| = n_s : p_h` — **the modulator carries the largest torque** (n_s/p_h ≈ 1.24
for 4/17/21). Kinematics (three-port constraint):

```
p_l·ω_l + p_h·ω_h = n_s·ω_mod
```

Modulator locked ⇒ ω_l/ω_h = −p_h/p_l = −4.25 (sign = direction reversal, handled in CAD).

## 2. The swing equation

With crank-side inertia `J_h` (ring + whatever is stiffly attached) and wheel-side `J_l`:

```
J_h·θ̈_h = T_in − T_po·sin δe
J_l·θ̈_l = −(p_l/p_h)·T_po·sin δe − T_load
```

Differentiating δe twice and collecting:

```
J̃·δ̈e + c̃·δ̇e + T_po·sin δe = F̃(t)

J̃ = (p_h²/J_h + p_l²/J_l)⁻¹                       (effective relative inertia)
F̃ = J̃·(p_h·T_in/J_h − p_l·T_load/J_l)             (effective drive)
```

This is **exactly the swing equation of a synchronous machine**. Stable equilibrium
δs = asin(F̃/T_po); pull-out at F̃ = T_po. `c̃` collects eddy damping in the gap
(asynchronous motion induces eddy currents — an induction-machine-like drag that is
*zero in sync*, which is why magnetic gears are so efficient at cruise).

**Numbers (v1 guess):** J_h ≈ 0.02 kg·m² (ring alone during fast transients — the
series element decouples the crank), J_l ≈ 0.004, p_h = 17, p_l = 4:
`1/J̃ ≈ 17²/0.02 + 4²/0.004 = 18 450` ⇒ swing frequency at 70% load
`ω_sw = √(T_po·cos δs/J̃) ≈ √(70·0.71·18450) ≈ 950 rad/s ≈ 150 Hz.`
Slip transients resolve in ~10 ms — invisible inside one pedal stroke (3 Hz ripple).
Integrator consequence: `dt ≤ 2×10⁻⁵ s` for a clean energy audit (spec O0).

## 3. Equal-area criterion (capture limit)

For steady load fraction `f = F̃/T_po < 1`, stable δs = asin f, unstable δu = π − asin f.
A transient may inject relative kinetic energy `½·J̃⁻¹`-normalized up to the decelerating
area:

```
E_crit(f) = ∫[δs→δu] (T_po·sin δ − F̃) dδ
          = T_po·[ 2√(1−f²) − f·(π − 2·asin f) ]
```

Checks: E_crit(0) = 2·T_po (full separatrix), E_crit(1) = 0. **E_crit is an energy and is
independent of J̃** — inertia determines how much δ̇ a given torque impulse produces
(Δδ̇ = ∫ΔF̃ dt·J̃⁻¹·…), not the ceiling itself. `ridesim::capture` implements exactly this
formula; the dynamic sims must agree with it (cross-validation of the ODE).

Per slipped pitch (δe advancing 2π), the drive pumps `2π·F̃` of energy into the relative
coordinate while damping removes `∮c̃·δ̇·dδe`; cyclic slipping settles where these balance.
Re-capture after a spike is guaranteed when post-spike f < 1 and swing energy has been
bled below E_crit — with c̃ from gap eddies this takes a few 10-ms swings ⇒ **recapture
well inside one crank revolution** (spec O2), *provided the post-spike load is inside
pull-out* — which is the CVT's job (§5).

## 4. Why the naive series-elastic spring fails — and the fix

Two-mass analysis, crank `J₁ ≈ 0.45 kg·m²` (cranks + rider limb reflection) against
ring+vehicle reflected `J₂ ≈ 0.55 kg·m²`, pedal ripple at 90 rpm is 2/rev ⇒
`ω_r = 18.8 rad/s`. Isolation of ripple requires the spring's two-mass natural frequency
`ω_n = √(k·(1/J₁+1/J₂))` to sit *below* ω_r/√2 ⇒ `k ≲ 44 N·m/rad`. But a magnetic sine
spring `T = T_max·sin(p_s·δ)` that must *carry* sprint torque (≥150 N·m) with p_s = 6 has
operating-point stiffness `k ≈ p_s·T_max·cos(p_s δ) ≈ 900 N·m/rad` — **20× too stiff**.
Conversely a spring soft enough to isolate cannot carry the mean torque: its own pull-out
is `T_max = k/p_s ≈ 7 N·m`. A linear/sine series-elastic element *cannot simultaneously
carry sprint torque and low-pass the pedal stroke.* (This kills spec O1 as originally
worded; changelog entry required.)

**The fix — torque-saturating ratchet buffer.** Make the series coupling a magnetic sine
coupling with pull-out `T_sat ≈ 0.95·T_po_referred`, i.e. *slightly weaker than the main
gear*. Below T_sat it is stiff (k ≈ 900 — mild ripple amplification ~1.1×, damped by
designed-in eddy plates c_series). Above T_sat it **ratchets pole-by-pole**: the crank
advances relative to the ring in 2π/p_s increments, each slip storing/releasing spring
energy and dissipating a bounded amount in eddy damping. The rider feels sustained
~T_sat resistance with a p_s-per-rev texture — the drivetrain's "tactile torque limit".
The buffer thus (a) protects the main gear (it always slips first, and its slips are
benign), (b) converts overtorque excursions into bounded, auditable energy events,
(c) needs no control system — it is passive physics.

## 5. Pedaling past the torque limit = trading torque for cadence

The gear caps *torque*, not *power*. OPSR's third layer exploits this:

- Sustained overtorque demand (sprint: rider mean 87 N·m, peaks 165 > T_po = 70) can
  never pass through synchronously at fixed ratio — excess torque would just spin up the
  crank (211 rad/s² at 95 N·m excess on 0.45 kg·m²).
- Instead, the **CVT port absorbs the excess as cadence**: the modulator motor lets the
  ratio fall (downshift) so crank speed rises while crank torque saturates at the buffer
  limit. Power through the gear: `P = T_sat·ω_crank`. At T_sat ≈ 66 N·m and 140 rpm
  (real sprint cadence) ⇒ **~970 W transmitted at 94% of pull-out** — a full sprint,
  *through* a 70 N·m-limited gear, with zero slip of the main mesh.
- Control law (implemented in `ridesim::cvt`): regulate measured load angle to
  `δ ≤ margin·δ_po` (margin 0.7) by commanding modulator speed; bandwidth needs only to
  beat the *pedal stroke* (3 Hz), not the swing dynamics (150 Hz) which self-stabilize.
- Modulator torque share is n_s/p_h ≈ 1.24 — the CVT motor must *react* large torque but
  at near-zero speed in cruise ⇒ small power. Loss model: P_cvt = |T_mod·ω_mod|·(1−η_mot)
  + controller idle. Spec G5 keeps the total ≤ 3%.

**Summary of OPSR (final architecture):**
1. *Saturating ratchet buffer* (passive) — peak texture + gear protection.
2. *CVT torque→cadence conversion* (active, 3 Hz bandwidth) — sustained overpower.
3. *Equal-area slip recovery* (intrinsic) — spike transients ratchet ≤ 1 pitch and
   re-capture in ~10 ms, energy audited.

## 6. What the rider experiences

Cruise: silent, contactless, > 97% efficient, zero cogging feel (G4). Hard stomp: pedals
firm up to ~66 N·m, then a smooth notched "give" (6/rev) while the bike surges as the
CVT converts the stomp into acceleration — like an automatic downshift with tactile
feedback. Nothing can be over-torqued, jammed, or stripped: the failure mode of a chain
(snap) is replaced by a bounded, reversible slip.

---
*Implementation notes for `ridesim`: state vector (θ_c, ω_c, δ_spring, θ_ring, ω_ring,
δe, δ̇e implied, v, ω_mod, x); RK4 fixed step dt ≤ 2e-5 s; every torque interface logs
work for the energy audit; slip detection = δe crossing (π − asin f) with hysteresis;
spring ratchet identical machinery with its own (p_s, T_sat, c_series).*
