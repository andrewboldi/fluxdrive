# FluxDrive Theory II — Mechanical Architecture

## Torque path (fully contactless, crank → road)

```
rider → cranks → [saturating magnetic ratchet buffer] → outer ring (p_h = 17)
      → ⟨flux-modulated gear, modulator = CVT port⟩ → inner rotor (p_l = 4)
      → [magnetic bevel coupling #1] → enclosed drive shaft (in chainstay)
      → [magnetic bevel coupling #2] → rear hub → wheel
```

No chain, no meshing teeth, no lubricated contact anywhere in the torque path.
Every coupling stage is magnetic and therefore also an overload-protection stage;
the buffer is deliberately the weakest link (T_sat ≈ 0.95·T_po referred), so slip
always happens at the designed, instrumented, benign element.

### Why the gear lives at the bottom bracket
Torque is largest at the crank (up to 200 N·m) and torque capacity buys radius —
the BB shell is the one place a 190 mm active diameter ring is natural. After the
4.25:1 step-up the drive shaft carries only ~8–16 N·m at 380–600 rpm, so the two
bevel couplings and the shaft stay compact inside a printed chainstay tunnel.
The gear's direction reversal (fixed-modulator topology) is cancelled by the
bevel-coupling orientation at the hub.

## The CVT port is also the power supply
The modulator ring is driven by a small outrunner (pseudo-direct-drive style).
Three jobs, one machine:
1. **CVT**: modulator speed sets effective ratio (3 Hz bandwidth suffices — it
   fights the pedal stroke, not the 150 Hz swing dynamics).
2. **Generator**: trickle-harvests during cruise to feed its own controller, the
   maglev axial coils, and sensors — the bike's mechatronics are self-powered;
   a supercap in the downtube buffers sprints.
3. **Launch assist logic** (software): standing-start stomp management (spec O4).

## Hybrid maglev bottom bracket ("frictionless pedaling")
- **Radial**: passive repulsive Halbach ring pairs on both sides of the spindle.
  Passive rings realistically deliver ~10⁵–10⁶ N/m; they carry *all* cruise loads
  (≤ 300 N radial) contact-free — true frictionless pedaling in normal riding.
- **Axial**: Earnshaw forbids full passive stability ⇒ small active axial coil
  pair + Hall sensors, budget ≤ 5 W (spec B3), powered by the CVT generator.
- **Peak loads**: 1.5 kN stomp transients exceed any bike-sized passive maglev;
  zirconia touchdown rings (0.3 mm clearance) take the hits for <1% duty. This
  is the same philosophy as turbomachinery maglev: levitate the duty cycle,
  touch down for the outliers. Honest spec: B2 reworded at v2 freeze.
- The gear's own negative radial stiffness (S2 number from magsim) adds to the
  maglev's load budget and is included in B2 verification.

## Structure
- **Frame**: pultruded carbon tubes (stock) joined by 3D-printed CF-nylon lugs —
  printed-lug construction is the proven route for printed bikes; every lug fits
  256³ mm. Chainstays are printed clamshell tunnels around the drive shaft.
  Downtube bay holds supercap + controller.
- **Gear housing**: two printed shells with labyrinth (contactless) sealing;
  magnet carriers are printed with press-fit pockets + retention lips + glue
  channels for stock sintered NdFeB blocks/arcs (never printed magnets in the
  torque path — bonded material is ~4× weaker in Br²).
- **Modulator cage**: printed cage with slots for laminated steel strips
  (eddy control by lamination, strips are stock shim steel).
- **Fairing**: printed pod around the BB gear, profile chosen by `lbmflow` (A2).

## CAD toolchain decision
**Blender 5.2.1 headless is the CAD engine**: parametric Python (`bpy` + exact
boolean modifiers, mm units) builds every part; the same source exports STLs
(M1–M3 checks via trimesh watertight + 3D-Print Toolbox), product renders for
the direct-viewing iteration loop, and the assembly animation (gear ratios as
rotation drivers). One toolchain from parameters to printable files to film.

## Printed part manifest (v1)
| # | Part | Qty | Notes |
|---|------|-----|-------|
| P1 | Outer ring magnet carrier | 1 | 2·p_h = 34 arc pockets |
| P2 | Inner rotor magnet carrier | 1 | 2·p_l = 8 pockets |
| P3 | Modulator cage | 1 | n_s = 21 steel-strip slots |
| P4 | Ratchet buffer rotor pair | 2 | p_s = 6 pole pairs each |
| P5 | Gear housing shells | 2 | labyrinth seal, fairing mount |
| P6 | Maglev BB carriers | 2 | Halbach ring pockets + coil bobbin |
| P7 | Bevel coupling halves | 4 | 45° magnet cones, 12 poles |
| P8 | Drive shaft tunnel (chainstay) | 2 | clamshell |
| P9 | Frame lugs | 6 | BB cluster, head, seat, dropouts ×2, junction |
| P10 | Fairing pod shells | 2 | lbmflow-optimized profile |
| P11 | Hub coupling housing | 1 | |
| P12 | Crank arms | 2 | CF-nylon, metal pedal inserts |
