# FluxDrive Device Acceptance Spec — v1

**TDD at the device level.** This spec was written before the design. Every criterion below maps to an executable simulation test (Rust `cargo test` or Python pytest). A criterion may only be changed with a written justification recorded in this file's changelog. Numbers marked `[R]` are to be confirmed/refined by the research phase before freeze; the *structure* of each test is frozen now.

## Reference conditions

| Symbol | Value | Meaning |
|---|---|---|
| m_sys | 85 kg | rider (75) + bike (10) |
| r_wheel | 0.335 m | 700c wheel radius (2.105 m circumference) |
| ρ_air | 1.225 kg/m³ | sea-level air |
| Crr | 0.005 | rolling resistance coeff `[R]` |
| CdA_base | 0.32 m² | baseline hoods position `[R]` |
| P_cruise | 250 W @ 90 rpm | endurance rider |
| P_sprint | 1000 W @ 110 rpm, 0.4 s spikes to 165 N·m crank | sprint case `[R]` |
| T_standstill | 200 N·m | standing-start stomp, worst case `[R]` |

## Acceptance criteria

### G — Gear (magsim + ridesim)
- **G1 Ratio correctness**: kinematic sim of chosen pole combo reproduces target ratio 4.25:1 within 0.1%, and the pole-pair rule `n_s = p_low + p_high` holds by construction.
- **G2 Pull-out torque**: static crank-side pull-out ≥ **70 N·m**, agreed within 15% by TWO independent models (Rust dipole-sum `magsim` vs Python `magpylib` analytic). `[R]` for final magnet spec.
- **G3 Torque curve shape**: transmitted torque vs load angle δ fits `T = T_po·sin(p_eff·δ)` with R² ≥ 0.98 in the operating region (validates the slip-dynamics model used everywhere else).
- **G4 Cogging**: peak cogging torque < **2%** of pull-out (enforced by pole-combination selection rule, verified numerically).
- **G5 Efficiency**: modeled drivetrain loss (eddy + windage + modulator control power) ≤ 3% at 200 W cruise.

### O — Overtorque architecture (ridesim)
- **O1 Peak shaving**: series-elastic buffer reduces crank torque peak/mean ratio from ≥1.8 to ≤1.35 at 250 W/90 rpm without adding >8° of low-frequency wind-up oscillation.
- **O2 Slip capture (equal-area)**: a 165 N·m, 0.4 s sprint spike at 80% load causes at most ONE pole-pitch slip and the gear **re-captures within 1 crank revolution**; energy lost to the slip event ≤ 15% of the spike's excess energy.
- **O3 CVT authority**: control law keeps load angle δ ≤ 0.7·(π/2p_eff) ("70% of pull-out") through an 8% grade transition at 250 W by ratio modulation alone (no slip events).
- **O4 Standing start**: 200 N·m stomp from 0 rpm does not cause runaway slip (pole-slipping cascade); system either holds or ratchets ≤ 3 pole pitches then captures.
- **O5 Graceful degradation**: with the CVT controller disabled (failure mode), a sprint causes cyclic slipping that limits transmitted torque but produces NO simulated component damage and rider cadence stays controllable (torque never drops to zero for > 50 ms).

### B — Bearings / frictionless path (magsim + analysis)
- **B1 Contactless torque path**: zero mechanical contact from crank input flange to wheel hub flange in the CAD assembly (verified geometrically: minimum clearance ≥ 0.8 mm across full rotation).
- **B2 Maglev BB radial capacity**: passive Halbach radial bearing supports 1.5 kN radial pedal load with ≤ 0.5 mm deflection `[R]`.
- **B3 Axial stability budget**: active axial stabilization steady-state power ≤ 5 W (Earnshaw makes fully-passive impossible; we prove the hybrid budget).
- **B4 Frictionless figure of merit**: total bearing + drivetrain drag torque at cruise ≤ 0.15 N·m referred to crank (≈ 1.4 W at 90 rpm).

### S — Structure (beam FEA, Python)
- **S1 Frame FoS**: printed frame sections FoS ≥ 2.0 under combined 1.5 kN pedal + 3g vertical road load (CF-nylon properties `[R]`).
- **S2 Gear carrier deflection**: magnetic radial attraction loads (computed by magsim) deflect printed carriers < 10% of air gap.

### A — Aero (lbmflow)
- **A1 Solver validation**: D2Q9 LBM reproduces cylinder drag Cd = 1.33 ± 10% at Re = 100 (canonical benchmark) before any fairing claim is made.
- **A2 Fairing gain**: best fairing profile reduces 2D section drag ≥ 15% vs unfaired gearbox pod section at design Reynolds number.

### M — Manufacturability (CAD checks, Python)
- **M1 Build volume**: every printed part fits 256×256×256 mm.
- **M2 Printability**: overhangs ≤ 60° from vertical or chamfered/split; all STLs watertight-manifold.
- **M3 Magnets**: only stock sintered magnet sizes (arc segments or rectangular blocks ≤ 40 mm) in printed pockets with retention features.

## Verification matrix

Every criterion gets: (a) an executable test, (b) a result artifact (plot/CSV) in `results/`, (c) a line in the paper. Tests are written FIRST and must fail before the design work makes them pass.

## Changelog
- v1 — initial spec, pre-research. `[R]` items to be frozen after research phase.
