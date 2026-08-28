# FluxDrive ⚡🚲

**A futuristic, 3D-printable bicycle drivetrain: coaxial magnetic gearing, contactless ("frictionless") pedaling, and an overtorque architecture that lets a rider pedal *past* the magnetic pull-out torque limit.**

Magnetic gears transmit torque through modulated magnetic fields with **zero tooth contact** — no lubrication, no wear, inherent overload protection. Their Achilles' heel: a hard **pull-out torque limit** (exceed it and the gear slips like a synchronous machine losing step). A human sprinter produces torque spikes (~150–200 N·m) far above what a bike-sized printed magnetic gear can hold (~70 N·m). FluxDrive's core contribution is a three-layer **Overtorque Phase-Slip Recovery (OPSR)** architecture that makes this limit survivable — and usable.

## The three layers of OPSR

1. **Series-elastic magnetic torsion buffer** — a contactless magnetic spring between crank and gear input shaves the 2-per-rev pedal torque peaks (peak/mean ≈ 1.9 → ≈ 1.3), raising usable rider power before any peak crosses the limit.
2. **Magnetic CVT downshift under load** — the flux modulator ring is speed-controlled (pseudo-direct-drive style), continuously varying effective ratio so demanded crank torque stays inside the stability margin. You pedal past the limit by *moving the operating point*, not by breaking physics.
3. **Phase-slip recovery ("magnetic ratchet")** — if a spike still exceeds pull-out, the gear slips exactly one pole pitch and *re-captures*, analyzed with the same equal-area criterion used for synchronous machine transient stability. Slip energy is buffered in rotor kinetic energy + the series-elastic spring instead of clutch heat. The gear is its own overload clutch.

## Subsystems

| Subsystem | What | Where |
|---|---|---|
| `sim/magsim` | Rust magnetostatics engine (dipole + harmonic models): torque curves, pull-out, cogging | Rust |
| `sim/ridesim` | Rust rider + drivetrain + slip-dynamics ODE simulator (equal-area capture, CVT control law) | Rust |
| `sim/lbmflow` | Rust D2Q9 lattice-Boltzmann CFD for fairing drag studies | Rust |
| `cad/` | Parametric CAD generators → STLs for 3D printing | Python |
| `blender/` | Blender scripts: assembly animation, renders | Python |
| `validation/` | Independent cross-checks (magpylib analytic fields vs magsim) | Python |
| `paper/` | Engineering paper: theory, design, simulation results | Typst/LaTeX |
| `tests/device_spec.md` | **Device-level TDD**: acceptance criteria the simulations must pass | — |
| `docs/` | Research notes and derivations | Markdown |

## Design snapshot

- Coaxial flux-modulated magnetic gear (Atallah–Howe topology), sintered NdFeB magnets in 3D-printed carriers
- Ratio ~4.25:1 nominal, CVT range via modulator control
- Hybrid maglev bottom bracket: passive Halbach radial repulsion + low-power active axial stabilization
- Frame: CF-nylon printed lugs + printed lattice sections, designed to FoS ≥ 2

## Status

Active development. See `bd ready` for the live issue backlog.

---
*Built with test-driven development at the device level: the acceptance spec (`tests/device_spec.md`) was written before the design, and every criterion is enforced by an executable simulation test.*
