# 08 — Electromagnetic Research Frontier Scan (2019–2026)

**Scope:** recent measured results and design rules from the magnetic-gear / electromagnetic literature that bear directly on FluxDrive (coaxial flux-modulated Atallah–Howe gear, ~4.25:1, 3D-printed carriers with sintered NdFeB inserts, ~70 N·m pull-out budget, crank 60–110 rpm / up to 200 N·m, magnetic CVT via modulator speed control, maglev bottom bracket).

**Bottom line up front**

- NASA Glenn's PT-1 was *literally a 4.25:1 concentric magnetic gear with 3D-printed nylon–carbon-fiber structure and off-the-shelf N52 magnets* (34 N·m measured, 20 N·m/kg). FluxDrive is re-running NASA Phase 1 at bicycle speeds — copy their lessons, skip their mistakes.
- Best measured coaxial-gear numbers at bike-relevant size: 147.8 N·m from a Ø150 mm × 114 mm Halbach gear (261 N·m/L *active region*, but only 73 N·m/L / 22.4 N·m/kg *full assembly*). A 70 N·m pull-out budget at bike scale is conservative-feasible; 150+ N·m is demonstrated but needs Halbach + 0.5 mm gaps + real engineering.
- Radial flux beats axial flux below roughly R_out ≈ 100 mm; axial only wins for large-radius pancakes and brings multi-kN unbalanced axial forces — poison for printed carriers and a maglev BB. **Stay radial.**
- Cycloidal/harmonic magnetic gears only pay off at ratios >16:1. At 4.25:1 the coaxial (CMG) topology is the right one; the 2–3× torque-density claims are real only at high ratio, in "active-region" accounting, and with severe mechanical complexity (orbiting rotor, unbalanced forces, bearing losses).
- Eddy losses at bike frequencies (<60 Hz electrical) are nearly irrelevant — NdFeB skin depth at 60 Hz is ~75 mm ≫ magnet size. NASA measured ≥99% efficiency below 1300 rpm output. Magnet segmentation is a nice-to-have, not a must, for FluxDrive; iron laminations in the modulator still matter.
- Magnomatics commercially markets the pull-out limit as a **"passively resettable torque fuse"** — external validation of FluxDrive's pedal-past-pull-out / phase-slip-recovery philosophy, and their MAGSPLIT is the commercial precedent for CVT-by-modulator-speed-control.

---

## 1. Core equations (plain-text / LaTeX)

**Flux-modulation condition and ratios** (Atallah–Howe topology; NASA EAP paper eq. 1–3):

```
q = p_l + p_h                       # modulator pole pieces = sum of low- and high-speed rotor pole pairs
G1 = p_l / p_h                      # modulator fixed (rotors counter-rotate)
G2 = 1 + p_l/p_h = q / p_h          # ring fixed (rotors co-rotate)
omega_1 * p_1 = omega_2 * n_2       # speed relation, inner rotor vs modulator cage
```

FluxDrive 4.25:1 candidate: p_h = 4, p_l = 17, q = 21 (17/4 = 4.25). Cogging-torque figure of merit: `C_T = gcd(2*p_1, n_2)` — minimize it; (4, 21, 17) gives gcd(8,21) = 1, the best possible (Wong/Bird eq. 2). With the modulator *driven* (CVT mode): `omega_h = (q*omega_mod - p_l*omega_l)/p_h` — continuously variable effective ratio.

**Torque-density definitions** (always state which one!):

```
VTD  = T_2 / (pi * r_o3^2 * d)      # active-region volumetric torque density [N·m/L]
T_m  = T_2 / (m_steel + m_magnet)   # active-region mass torque density [N·m/kg]
Full-assembly values are 2.5–3.6x WORSE (see §3, §4).
```

**Ideal geometric scaling** (Gardner/Johnson/Toliyat 2018, Table I; σ = air-gap shear stress):

| Quantity | Radial flux | Axial flux |
|---|---|---|
| Air-gap area | ∝ R_out·H | ∝ R_out² |
| Lever arm | ∝ R_out | ∝ R_out |
| Torque | ∝ σ·R_out²·H | ∝ σ·R_out³ |
| Volume | ∝ R_out²·H | ∝ R_out²·H |
| VTD | ∝ σ (size-invariant) | ∝ σ·R_out/H (grows with radius) |

Measured/FEA air-gap shear stress for both topologies: σ ≈ 60–110 kN/m² (NdFeB N42, 1 mm gaps). Quick sanity check for FluxDrive: T ≈ 2π·R²·L·σ ⇒ at R = 65 mm, L = 40 mm, σ = 70 kN/m²: T ≈ 2π·0.065²·0.04·70000 ≈ 74 N·m — the ~70 N·m budget is exactly what the shear-stress law predicts at BB-shell scale.

**Aircraft mechanical-gearbox benchmark** (NASA fit over 61 TRL-9 transmissions): `tau/m = 18.3 * tau^0.193` (N·m/kg with τ in N·m). At τ = 70 N·m → ~42 N·m/kg. NASA PT-2 measured 44.7 N·m/kg — magnetic gears are already *mass-competitive with aerospace mechanical gearboxes at the ~100 N·m scale* (they lose badly at 10⁴–10⁶ N·m).

**Eddy loss in a thin conductor (resistance-limited regime):**

```
P_v = (pi^2 / 6) * sigma_e * f^2 * B_pk^2 * t^2     # W/m^3, t = segment dimension transverse to flux
delta = sqrt(rho / (pi * f * mu_0 * mu_r))          # skin depth
```

Segmenting into n pieces: t → t/n so P → P/n² *ideally*; 3-D return paths degrade this (measured: 14 segments/pole → only ~8× reduction, not 196×; see §8). Valid only while t ≪ δ — for NdFeB (ρ ≈ 1.4 µΩ·m, µ_r ≈ 1.05), δ(60 Hz) ≈ 75 mm, δ(1 kHz) ≈ 19 mm. FluxDrive electrical frequencies: f = p·n/60 ≤ 17×110/60 ≈ 31 Hz (low-speed side), ≈ 4×470/60 ≈ 31 Hz (high-speed side) → always resistance-limited, absolute losses small.

**Temperature derating (NdFeB):** NASA PT-4 2-D estimates: 370 N·m @ 20 °C → 303 N·m @ 80 °C → 270 N·m @ 100 °C, i.e. **~18% pull-out torque loss from 20→80 °C** (≈ −0.3%/°C). FluxDrive's slip threshold must be specified at max summer + slip-heating temperature, not bench temperature.

---

## 2. NASA Glenn magnetic gearing series (PT-1 → PT-4, 2017–2023) — the single best dataset

Source: Scheidler, *Progress in Magnetic Gearing for Aeronautics and Space Applications*, NASA GRC, Energy & Mobility Conf. 2023 (NTRS 20230012310) + Scheidler/Asnani/Tallerico, *NASA's Magnetic Gearing Research for Electrified Aircraft Propulsion*, EATS 2018 (NTRS 20180005333).

| | PT-1 (2017) | PT-2 (2017) | PT-3 (2018-19) | PT-4 (2019-23) |
|---|---|---|---|---|
| Type | concentric | concentric | concentric | concentric |
| Gear ratio | **4.25:1** | 4.83:1 | 4.83:1 | 12.1:1 |
| Rated output speed | 4500 rpm | 4500 rpm | 4500 rpm | 661 rpm |
| Peak output torque | 34 N·m | 124 N·m | 112 N·m | 226 N·m (est.) |
| Size (Ø×L) | 141×92 mm | 154×114 mm | 154×114 mm | 212×131 mm |
| Specific torque | 20 N·m/kg | 46 N·m/kg | 42 N·m/kg | 49 N·m/kg (est.) |
| Efficiency (100 rpm, 85% T) | – | 99.1% | **99.6%** | – |

Pre-NASA state of the art was ~20 N·m/kg; NASA roughly 2.5×'d it. Magnetic gear ≈ 35 kW/kg vs motor ≈ 10 kW/kg — gearing a small motor beats direct drive on mass.

**PT-1 details (the FluxDrive twin):** off-the-shelf N52 magnets, electrical-steel laminated modulator, **3D-printed nylon–carbon-fiber structures**. Measured 34 N·m vs 53 N·m predicted by 2-D FEA (−35%, low aspect ratio → axial end-effect leakage). Mass 1.7 kg = 1.0 active + 0.7 structural (41% structural). Lessons stated verbatim: (a) gaps between off-the-shelf magnets in the sun cost real torque — custom annular-wedge magnets fixed this in PT-2; (b) magnetic forces during assembly are significant — assemble sun/modulator/ring as separate subassemblies, integrate with shims; (c) "printing tolerances are inadequate for minimizing physical gaps" — printed structure OK for strength, not for defining air-gap surfaces.

**PT-2:** custom wedge magnets, 6-segment Halbach arrays (up from 4), thinner pole pieces, dedicated modulator-lamination containment: 128 N·m preliminary measured (178 N·m 2-D), 44.7 N·m/kg, 2.87 kg total (again 41% structural). Also reported as 162 N·m/L volumetric with **all-plastic housing**.

**PT-3 (efficiency champion):** 0.1 mm iron laminations, **1 mm laminated magnets → only 80% magnet fill factor** (torque penalty accepted for loss reduction). Measured: >99% efficiency up to ~1300 rpm output, ~97% at max torque & speed, extrapolated >99.5% at low speed. *FluxDrive's wheel-side speeds (≤ ~470 rpm) sit entirely in the ≥99% region.*

**Scaling studies (Tallerico 2019, NTRS 20190033112 & 20190032274):** 2-D electromagnetic specific torque scales ~linearly with radius (150→210 N·m/kg over R = 70→140 mm, ratios 4–6 best; ratio 10 worst). **Air-gap sensitivity: ~225 N·m/kg @ 0.25 mm gap → ~130 @ 1 mm → ~60 @ 2.5 mm** — the single steepest design lever. OH-58 case study: mechanical planetary active mass 8.0 kg vs magnetic 22 kg with 1 mm gaps/2 mm modulator bridge → 12.0 kg at 0.5 mm/1.3 mm → 8.7 kg at 0.25 mm — i.e., magnetic gears are 2–3× heavier than mechanical *today*, closing to parity with manufacturing improvements. Axial flux leakage typically costs 20–40% of 2-D-predicted torque (Gerber & Wang correction).

**MDECE (2020–2024, lunar actuators):** cycloidal MG proof-of-concept; 0.25 mm air gaps "achievable with careful structural & mechanical design"; bearings, not magnetics, are the life-limiting component — an argument *for* FluxDrive's maglev BB direction. SBIR spinouts: FluxWorks LLC (Praslicka, ex-TAMU) and FluxMagic/Portland State.

---

## 3. Halbach segment arrays — measured gain and cost

- **Wong/Bird et al. 2019 (measured, closest to bike scale):** 5.67:1, (p1,n2,p3) = (3,17,14), NdFeB **N48**, M19 laminations, active OD 120 mm × 50 mm stack, **0.5 mm air gaps**. 3-D FEA 160.6 N·m / 284 N·m/L; **measured 147.8 N·m / 261.4 N·m/L active-region** (−8%), 39.9 N·m/kg active mass, 57.2 N·m/kg per magnet mass. **Full assembly: Ø150 × 114.3 mm, 6.6 kg → 73.2 N·m/L and 22.4 N·m/kg.** Measured torque ripple 11% vs 0.58% predicted (suspected eccentricity/vibration; odd modulator count n2 = 17 gives unbalanced radial force — they recommend an even cage-bar count). 2-D FEA overpredicts ~25% when stack length ≲ radius.
- Halbach vs conventional radial magnetization: Jian et al. 2009 (4.25:1 Halbach CMG!) 155.8 N·m / 108 N·m/L; Jing 2016 (4.25:1) 168 N·m / 129.8 N·m/L; case-study numbers across the literature: **+14.3% torque density and −13.5% transmission error** for Halbach vs standard; a dual-layer (Halbach + radial) outer rotor: **+22.2% torque / +27.4% torque density** (2024); flux-focusing Halbach ("FFH-CMG") variants claim more. Uppalapati flux-focusing (non-Halbach) measured 239 N·m/L.
- Best 2-D-optimization claims 2024: 411 kN·m/m³ VTD and 830 kN·m/m³ per PM volume (dual-flux-modulator, double-layer PM inner rotor, surrogate-optimized — unbuilt).
- **Cost/complexity of Halbach:** segment count multiplies. NASA PT-2 went 4 → 6 magnets per array; a 2-segment/pole Halbach doubles magnet piece count and every piece has a distinct magnetization direction (procurement + orientation-keyed carrier pockets + glue fixtures). Assembly forces between adjacent segments are repulsive — carriers must retain magnets against both centrifugal and inter-magnet forces. Benefit stack: ~+15–25% torque, near-sinusoidal field (lower ripple & harmonic loss), and **little/no back iron needed** (one-sided field) — attractive for FluxDrive: replaces back-iron steel with printed polymer, saving mass exactly where a 3D-printed design wants to.
- Practical FluxDrive translation: at a 70 N·m budget, Halbach either buys ~15–25% torque margin in the same envelope or lets you open air gaps from 0.6 → ~0.8 mm at constant torque (printing/assembly tolerance relief). N48–N52 sintered segments; 2 segments/pole is the cost-sane sweet spot at p_l = 17 (68 outer pieces vs 34).

## 4. Axial-flux vs radial-flux for the pancake form factor

Gardner/Johnson/Toliyat 2018 (TAMU), 34,560 axial + 5,928 radial 3-D FEA designs, N42, 1 mm gaps, Somaloy modulators, ratios ~4/9/16:

- **Crossover:** axial VTD grows ~linearly with R_out; radial VTD ~flat. At R_out = 50 mm axial ≈ 80 kN·m/m³ < radial ≈ 110–125; crossover ≈ **R_out ~100 mm**; at R_out = 200 mm axial ≈ 410 vs radial ≈ 230–250 kN·m/m³.
- Both topologies achieve the same air-gap shear stress (~60–110 kN/m²); axial's win is purely the R² gap area. Ratio 4 outperforms 9 and 16 for both (FluxDrive's 4.25 is in the sweet spot). Axial optimum at radii-ratio ≈ 0.5 (VTD) or ~0.75 (PM-mass economy).
- **Axial killer flaw:** net axial magnetic forces on the rotors *cannot be canceled by symmetry* — 5–15 kN at stall for R = 100–200 mm designs. Radial designs cancel radial forces by symmetry. For a maglev BB whose levitation budget is tens–hundreds of N, a multi-kN magnetic axial preload is disqualifying; it also demands stiff (heavy, machined) rotor structure — exactly what a printed carrier can't provide.
- Authors' verdict: radial "likely superior in most applications — simpler construction, higher performance at most form factors"; axial only for large-R/short-L pancakes or power through a flat barrier.
- **FluxDrive decision:** a BB-integrated gear (OD 120–160 mm) is firmly radial-flux territory. Axial only re-enters if the gear migrates to a full hub-shell-diameter disc (R_out ≳ 120 mm) *and* an axial-force management scheme (dual mirrored axial gaps) is accepted — not compatible with printed tolerances at v1.

## 5. Cycloidal / harmonic magnetic gears — how real is 2–3×?

- Topology guidance from the TAMU/FluxWorks group: **CMGs (coaxial) are optimal at low ratios; cycloidal (CyMG) at ratios > ~16:1.** At 4.25:1 a cycloidal gear is the wrong tool — no decision needed beyond that.
- Claimed densities are real but conditional: flux-focusing CyMG 291 N·m/L (2-D calc, Ø228 mm, −25:1); measured CyMGs: 106.8 N·m/L (radial), 91.5 N·m/L (axial, R = 203 mm); GA-optimized 2-D VTD ≈ 250 kN·m/m³ @ 20:1 falling to ~160 @ 80:1; PM specific torque up to ~380 N·m/kg-PM @ 50–60:1. Praslicka 2021: practical 50:1 CyMG with balanced off-axis moments, high specific torque, for lunar robots.
- **Hasanpour 2023 (measured):** first consequent-pole CyMG, 20:1, Ø81 mm: slip torque 3.4 N·m measured vs 3.55 3-D FEA (95%); companion SPM CyMG 5.1 vs 5.36 N·m (9.7 N·m/kg). No-load losses ~25% of max power in this small build (solid back iron + sleeve bearings on the pins) — the complexity tax in microcosm: eccentric orbiting rotor, unbalanced magnetic force (mitigated by flux-shield "moon" structures or twin opposed sections), pin/bearing losses, and at high ratios "very large numbers of very small PMs."
- Harmonic MG (Rens/Atallah 2010; eccentric-harmonic 2023): ~150 kN·m/m³ claims, ratios to 360:1; 2023 EHMG with Halbach + shaped PMs raised torque 38.0 → 50.4 N·m (+33%) in FEA. Still lab-stage, eccentric motion, no measured full-efficiency prototypes at scale.
- Verdict for FluxDrive: 2–3× torque-density headlines are (a) at high ratio, (b) usually 2-D "active-region" numbers, (c) paid for with mechanical complexity that defeats FluxDrive's "contactless simplicity" thesis. Ignore for the 4.25:1 main stage; revisit only if a >16:1 single-stage hub variant ever appears.

## 6. Pseudo-direct drive & the commercial state of the art (Magnomatics et al.)

- **PDD = magnetic gear + PM machine merged** (stator winds around the gear; windings couple to the high-speed harmonic). Magnomatics (Sheffield, UK; Atallah co-founder) claims: highest continuous torque density of any air-cooled machine class, **~60% of the size/mass of an equivalent PM machine**, aerospace actuator line **>30 N·m/kg**, and — key for FluxDrive — an inherent, **passively resettable torque fuse**: overload slips magnetically and re-engages, no damage. Products: PDD Traction Motor (rail), PDD Aerospace Actuator (flight-control, aerospace AS9100 cert), PDD Marine Turbine (15 kW & 25 kW ROV thrusters delivered for SMD Quantum/Atom), utility-scale wind generator program (£5M UK grant, "Winder" project; LCOE reduction claimed), UAM air-cooled propulsion motor.
- **MAGSPLIT** (Magnomatics): magnetic power-split / CVT for hybrid drivetrains — *the* commercial precedent for FluxDrive's magnetic CVT via modulator speed control. Same physics: torque ratio fixed by pole counts, speed ratio varied by driving the third member electrically.
- **FluxWorks LLC** (TAMU spinout, Praslicka; NASA SBIR 2023-24 with UT Dallas): commercializing magnetic gears/geared actuators for aerospace, space, dusty/cryogenic environments. FluxMagic Inc. + Portland State (Bird): NASA SBIR aircraft drivetrain (12.1:1). Ecosystem is real but young; nobody ships a bicycle-scale product — open field for FluxDrive.
- Sanity for FluxDrive's marketing: quote *full-assembly* N·m/kg like Magnomatics' >30 N·m/kg aero number, not active-region figures.

## 7. Superconducting magnetic gears — note and dismiss

Bulk-HTS modulator CMGs (Jian & Chau 2010 onward) and HTS-excited designs measurably help: an HTS-toothed modulation ring raised max torque **1.73×** vs the ferromagnetic-toothed equivalent; trapped-field HTS bulks can replace PMs entirely. All require cryogenics (≤77 K LN₂, typically 20–30 K for useful trapped fields), cryostats in the air gap (fattening the effective gap), and quench management. Mass/cost/complexity are absurd at bicycle power (a cryocooler alone exceeds the whole drivetrain mass budget). **Dismissed for FluxDrive; cite only as evidence that modulator permeability/leakage is a first-order torque lever** — which cheap means (thinner bridges, laminated high-saturation steel, better bridge topology) also exploit.

## 8. Eddy-loss mitigation — magnet segmentation rules

- Regime check first: segmentation follows the 1/n² law only when segment size ≪ skin depth (resistance-limited). **With skin effect active, segmentation can *increase* loss.** NdFeB δ(60 Hz) ≈ 75 mm → FluxDrive is deep in the safe regime; conversely absolute eddy loss at ≤31 Hz is small (P ∝ f²).
- Measured/modelled reduction factors: 14 cuboidal segments/pole in an axial-flux PM machine → **8× loss reduction** (not the ideal 196× — 3-D return paths); NASA PT-3's 1 mm magnet laminations + 0.1 mm iron laminations bought 99.6% peak efficiency but cost **20% magnet fill factor** (torque!). Loss split: in magnetic gears, *PM eddy loss dominates total loss* as speed rises (TAMU 2018); iron loss secondary.
- Direction matters: for the same piece count, segmenting along the direction that shortens the eddy loop transverse to flux (usually axial slicing in radial machines) beats transverse slicing; "partial segmentation" (segmenting only high-loss regions, e.g. surface layer or pole edges) captures most of the benefit at a fraction of the cost.
- **FluxDrive rules:** (1) don't laminate magnets at v1 — pole-passing ≤ ~35 Hz makes it a rounding error; keep the option for the CVT high-slip corner (modulator driven fast) where local frequencies can triple; (2) laminate the modulator pole pieces (0.2–0.35 mm M-series steel or SMC) — mandatory, they see the full modulation spectrum; (3) if magnets ever run hot in slip events, 2–3 axial segments per magnet is the 80/20 fix; (4) polymer carriers already eliminate the "conductive structure" loss channel NASA worked to remove (Garolite rods, non-conductive end plates in Wong/Bird; plastics/composites in NASA list).

## 9. 3D-printed electromagnetics — what's real

- **Printed structure + sintered inserts (FluxDrive's architecture) is validated:** NASA PT-1 (nylon-CF printed carriers, N52 inserts, 34 N·m) and PT-2 (162 N·m/L, 44.7 N·m/kg, "all-plastic housing"). Caveat verbatim: printing tolerances can't define minimum air gaps — machine or insert the gap-facing surfaces, or budget ≥0.8–1 mm gaps and pay ~40% specific-torque penalty vs 0.5 mm (NASA air-gap curve, §2).
- **Fully printed magnets are not competitive for torque:** ORNL BAAM bonded NdFeB (65 vol% powder in PA12): Br = 0.51 T, Hci = 688 kA/m, (BH)max = 43.5 kJ/m³, density 4.8 g/cm³ — vs sintered N48: Br ≈ 1.4 T, (BH)max ≈ 380 kJ/m³. Torque ∝ ~Br² → printed magnets give ~13% of sintered torque. Use printed/bonded material only for field-shaping trim, sensors, or the maglev BB's passive bias features.
- **Printed Halbach and field-shaped magnets** (TU Wien, Huber et al., arXiv 1605.07309, 1702.01968, 2201.07111): FDM-printed polymer-bonded magnets with topology-optimized shapes and in-situ alignment of anisotropic powder in an external field during printing (raises Br toward ~0.7 T locally). Enables magnetization patterns impossible to assemble from blocks — relevant to FluxDrive for the maglev BB's passive gradient rings and for prototyping field geometry cheaply before ordering sintered segments.
- Soft-magnetic printing: SLM FeSi (Elkem/SOMA project powders), printed SMC and FeSi cores reviewed 2020–2023 (losses still 2–5× laminations at 50–400 Hz; brittleness improving). MIT 2026: first *fully* printed motor (multi-material extrusion: conductive ink + soft & hard magnetic pellets + polymers) — milestone, but mN·m-class; a decade from drivetrain torque. MDPI *Machines* 2025 "Experiments in 3D Printing Electric Motors" documents practical FDM motor builds. For FluxDrive: print carriers/housings (proven), buy sintered magnets and laminations (proven), don't print flux-carrying parts at v1.

---

## 10. Five most important papers — technical summaries

**1) Scheidler, J.J. (NASA GRC), "Progress in Magnetic Gearing for Aeronautics and Space Applications," Energy & Mobility Conf., Sept 2023 (NTRS 20230012310).**
The consolidated NASA scorecard. Four concentric prototypes: PT-1 (4.25:1, Ø141×92 mm, 34 N·m, 20 N·m/kg), PT-2 (4.83:1, 124 N·m, 46 N·m/kg), PT-3 (4.83:1, 112 N·m, 42 N·m/kg, measured 99.6% peak efficiency; >99% up to 1300 rpm output via 0.1 mm iron + 1 mm magnet laminations at 80% magnet fill), PT-4 (12.1:1, Ø212×131 mm, est. 226 N·m and 49 N·m/kg, self-cooled, magnets ≤80 °C). Key curves: 2-D EM specific torque vs air gap (~225 N·m/kg @ 0.25 mm → ~60 @ 2.5 mm); specific torque vs radius (linear, ratios 4–6 optimal); NdFeB torque −18% from 20→80 °C; OH-58 comparison showing magnetic active mass 2–3× mechanical today (22 kg → 8.7 kg pathway via 0.25 mm gaps + 1.3 mm modulator bridges vs 8.0 kg mechanical). Concludes magnetic gears feasible for aero, hinging on gap/modulator manufacturing; bearings (not magnets) limit life — supports FluxDrive's maglev BB.

**2) Wong, H.Y., Bird, J.Z., et al., "A High Torque Density Halbach Rotor Coaxial Magnetic Gear," IEEE ECCE 2019 (NSF PAR 10106669).**
Best measured coaxial gear at near-bike scale. 5.67:1, (3,17,14) poles, N48, 0.5 mm gaps, active Ø120×50 mm: measured 147.8 N·m pull-out (8% under 3-D FEA's 160.6), 261.4 N·m/L active VTD, 39.9 N·m/kg active, 57.2 N·m per kg of magnets — but the **full assembly** (Ø150×114 mm, 6.6 kg) delivers 73.2 N·m/L and 22.4 N·m/kg, a 3.6× active-to-assembly gap that FluxDrive must design against. Torque ripple measured 11% vs 0.58% predicted (odd modulator count → unbalanced radial force, suspected eccentricity); recommendation: even cage-bar counts. Documents Garolite (G10) modulator rods and non-conductive end plates to kill stray eddy paths, and 2-D vs 3-D overprediction (~350 vs 284 N·m/L at L≈R).

**3) Gardner, M.C., Johnson, M., Toliyat, H.A., "Comparison of Surface Permanent Magnet Axial and Radial Flux Coaxial Magnetic Gears," IEEE (TAMU), 2018.**
Parametric 3-D FEA over 40k designs (N42, 1 mm gaps, ratios ~4/9/16, R_out 50–200 mm). Establishes the scaling table (§1): axial VTD ∝ σR/H grows linearly with radius, radial VTD ≈ flat; crossover near R_out ≈ 100 mm (at 50 mm radial wins ~110–125 vs ~80 kN·m/m³; at 200 mm axial wins ~410 vs ~240). Both reach the same air-gap shear stress (60–110 kN/m²), and low ratios (4) maximize both VTD and PM economy. Decisive negative for axial at FluxDrive scale: unbalanced net axial rotor forces of 5–15 kN at stall that symmetry cannot cancel, vs radial designs whose radial forces cancel. Verdict: radial for most applications; axial only for very-large-R pancakes or through-wall drives.

**4) Scheidler, J.J., Asnani, V.M., Tallerico, T.F., "NASA's Magnetic Gearing Research for Electrified Aircraft Propulsion," AIAA/IEEE EATS 2018 (NTRS 20180005333).**
The build report FluxDrive should treat as a construction manual. PT-1: N52 off-the-shelf magnets + laminated modulator + 3D-printed nylon-CF carriers; measured 34 N·m (−35% vs 2-D, end effects at low L/D); lessons: inter-magnet gaps cost torque; assembly forces demand subassembly integration with shims; printed tolerances can't set air gaps. PT-2: custom annular-wedge magnets, 6-segment Halbach arrays, thinner pole pieces, modulator containment → 128 N·m preliminary, 44.7 N·m/kg (vs 61 predicted), 41% structural mass in both prototypes. Fits aircraft-gearbox trendline τ/m = 18.3τ^0.193 within 13% at ~100 N·m scale — magnetic gears are mass-competitive precisely at FluxDrive's torque class. Describes the E-Drives rig: ±0.02% torque accuracy needed because 99%-class efficiency is unresolvable with ordinary sensors — FluxDrive dyno note.

**5) Hasanpour, S., Johnson, M., Gardner, M.C., Praslicka, B., Goodarzi, A., Toliyat, H.A., "Construction of Radial Flux Cycloidal Magnetic Gears with Reduced Permanent Magnet Piece Count Using Consequent Poles," IEEE IEMDC 2023.**
Reality-check on the "2–3× torque density" cycloidal claims. First consequent-pole CyMG prototype (20:1, Ø81 mm, N50/N52H): measured slip torque 3.4 N·m = 95% of 3-D FEA; companion SPM CyMG 5.1 N·m (9.7 N·m/kg). GA-optimization curves show CyMG VTD ~250 kN·m/m³ at 20:1 declining with ratio, PM specific torque peaking ~380 N·m/kg-PM near 50–60:1 — i.e., cycloidal wins only at ratios ≫ FluxDrive's 4.25. Honest accounting of the complexity tax: no-load losses ≈25% of max power in this build (solid back iron + sleeve bearings on eccentric pins), unbalanced magnetic force handling, halving of PM piece count via consequent poles as a cost lever. Cites 94% efficiency as the best prior measured CyMG (Jørgensen).

---

## 11. Design implications for FluxDrive (actionable)

1. **Ratio & pole selection:** 4.25:1 = 17/4 with q = 21 modulator poles; gcd(8,21)=1 minimizes cogging; ratios 4–6 are the literature's torque-density sweet spot. Use an **even modulator count** alternative (e.g., 18/4 → q=22, ratio 4.5, gcd(8,22)=2) if the 11%-ripple/unbalanced-force lesson from Wong/Bird outweighs exact-ratio preference — worth a trade study.
2. **Topology locked: radial flux.** BB-scale OD < 200 mm is below the axial crossover, and axial's multi-kN unbalanced axial force is incompatible with printed carriers and the maglev BB force budget.
3. **Pull-out budget confirmed:** σ ≈ 60–110 kN/m² ⇒ ~70 N·m at R≈65 mm, L≈40 mm active. To let riders deliver 200 N·m at the crank, the buffer chain (series-elastic + CVT downshift + phase-slip) is not optional physics-wise — no bike-scale magnetic gear in the literature carries 200 N·m directly (best measured anywhere near the size: 147.8 N·m at 6.6 kg).
4. **Air gap is the steepest lever:** every 0.5 mm of gap costs ~30–40% specific torque (NASA curve). Printed parts must not define gap surfaces (NASA PT-1 lesson) — use machined/molded inserts or post-machined bores; target 0.6–0.8 mm mechanical gap at v1, 0.4–0.5 mm at v2.
5. **Magnet retention ≠ air gap:** design carriers so retention lips sit outside the active gap (Gardner note: retention sleeves on high-speed rotor fatten the effective gap). Wedge-shaped custom magnets (PT-2) recover the inter-magnet gap losses of rectangular off-the-shelf stock — at bike volumes, custom wedge N48/N52 is a modest cost.
6. **Halbach: yes, 2 segments/pole.** +15–25% torque or equivalent gap relief; eliminates most back iron (mass where FluxDrive wants none); accept ~2× magnet piece count. Orientation-keyed printed pockets make assembly of 68+ distinct-direction segments tractable — a genuine advantage of printed carriers.
7. **Expect the 2-D→3-D→measured haircut:** −20–40% from 2-D FEA (axial leakage at low L/D), then −5–10% build tolerance (Wong −8%, Hasanpour −5%, NASA PT-1 −35% at very low L/D). Design in 3-D FEA and add ≥15% torque margin over the CVT downshift threshold.
8. **Quote full-assembly numbers:** structural mass ran 41% at NASA and 3.6× active-to-assembly VTD penalty at Wong/Bird. A FluxDrive gear with 70 N·m pull-out at ≤3 kg full assembly (≥23 N·m/kg) would match the measured state of the art; ≤2 kg would beat it.
9. **Thermal derating:** slip threshold falls ~0.3%/°C (−18% at 80 °C). Specify pull-out at 60 °C worst case; instrument slip events (each slip dumps heat into magnets/modulator); demagnetization check at 100–120 °C with chosen grade (use ≥ N48**SH**-class temperature grade for slip-heavy duty).
10. **Efficiency story is safe:** at ≤31 Hz electrical, expect ≥99% magnetic efficiency (NASA PT-3 region) — better than a clean chain (~97–98.5%) and it never degrades with dirt. Laminate modulator poles (0.2–0.35 mm); skip magnet lamination v1 (80% fill-factor penalty not worth it below ~50 Hz); no conductive printed inserts near gaps.
11. **Overload = feature:** adopt Magnomatics' "passively resettable torque fuse" language and MAGSPLIT's variable-speed third-member operation as commercial precedent for slip-and-recover + magnetic CVT. Phase-slip recovery control should re-engage within one pole pitch (2π/17 ≈ 21° on the ring) — instrument with a low-rate encoder per rotor.
12. **Bearings are the reliability frontier** (NASA MDECE conclusion) — the maglev BB attacks the right problem; but keep the multi-kN radial magnetic stiffness of the gear itself in the maglev load budget (gear eccentricity creates net radial pull ∝ offset; Wong's ripple anomaly shows what eccentricity does).
13. **Simulation stack:** 2-D fast iteration (FEMM/analytical subdomain models), mandatory 3-D check (end effects), magpylib/Radia for maglev + Halbach field shaping, RL/GA optimization loops are now standard practice (Machines 2025 RL-FEA paper).

## 12. References

**NASA NTRS (all with direct PDFs)**
- Scheidler 2023, Progress in Magnetic Gearing for Aeronautics & Space — https://ntrs.nasa.gov/api/citations/20230012310/downloads/2023EnergyMobilityScheidlerFinal.pdf
- Scheidler/Asnani/Tallerico 2018, NASA's Magnetic Gearing Research for EAP — https://ntrs.nasa.gov/api/citations/20180005333/downloads/20180005333.pdf
- Tallerico 2019, Results of NASA's Magnetic Gear Program to Date — https://ntrs.nasa.gov/api/citations/20190033112/downloads/20190033112.pdf
- Tallerico/Scheidler 2019, Magnetic Gears and Their Structural Limitations — https://ntrs.nasa.gov/api/citations/20190032274/downloads/20190032274.pdf
- Tallerico 2019, Design of a Magnetic Gear for NASA's Vertical Lift Quadrotor (PT-4) — https://ntrs.nasa.gov/api/citations/20190030679/downloads/20190030679.pdf
- Tallerico 2019 (companion, 20190030282) — https://ntrs.nasa.gov/api/citations/20190030282/downloads/20190030282.pdf

**Halbach / coaxial**
- Wong, Bird et al. 2019, High Torque Density Halbach Rotor CMG — PDF: https://par.nsf.gov/servlets/purl/10106669 (also PDXScholar: https://pdxscholar.library.pdx.edu/ece_fac/486/)
- Li, Bird et al., Coaxial gearbox with flux-concentration Halbach rotors (wind demo) — https://www.osti.gov/pages/servlets/purl/1505685
- MDPI Computation 12(5):88, 2024, Halbach CMG analytical 2-D model — https://doi.org/10.3390/computation12050088
- Johnson/Gardner/Toliyat 2015, Axial-field MG with Halbach arrays — https://bpb-us-e2.wpmucdn.com/labs.utdallas.edu/dist/9/93/files/2021/09/Axial-Halbach-IEMDC-2015.pdf

**Axial vs radial / scaling**
- Gardner/Johnson/Toliyat 2018, Axial vs Radial SPM CMGs — https://bpb-us-e2.wpmucdn.com/labs.utdallas.edu/dist/9/93/files/2021/09/AvR.pdf
- Johnson et al. 2016, Axial Flux Magnetically Geared Generator — https://bpb-us-e2.wpmucdn.com/labs.utdallas.edu/dist/9/93/files/2021/09/AFMGM.pdf

**Cycloidal / harmonic**
- Hasanpour et al. 2023, CP Cycloidal MG construction (IEMDC) — https://bpb-us-e2.wpmucdn.com/labs.utdallas.edu/dist/9/93/files/2023/05/CP-CyMG-IEMDC-2023.pdf
- Gardner et al., 50:1 multisection cycloidal — https://labs.utdallas.edu/epl/files/2021/09/MultisectionCycloidal.pdf
- Eccentric harmonic MG 2023 — https://www.mdpi.com/2076-0825/12/6/248

**Recent optimization (2024–2026)**
- RL optimization of CMG geometry, Machines 13(12):1143 — https://doi.org/10.3390/machines13121143
- CMG design vs thermal demagnetization, Actuators 15(1):59 — https://doi.org/10.3390/act15010059
- Double-V hybrid PM CMG, JEET 2025 — https://link.springer.com/article/10.1007/s42835-025-02575-x

**Commercial**
- Magnomatics PDD — https://www.magnomatics.com/pseudo-direct-drive ; aerospace >30 N·m/kg — https://www.magnomatics.com/aerospace ; UAM motor — https://www.compositesworld.com/news/magnomatics-designs-air-cooled-propulsion-motor-for-uam ; offshore wind grant — https://www.rivieramm.com/news-content-hub/news-content-hub/magnetic-drive-for-offshore-wind-turbines-secures-grant-funding-61341
- FluxWorks LLC / NASA SBIR — https://sbir.nasa.gov/SBIR/abstracts/23/sbir/phase1/SBIR-23-1-Z13.05-1119.html

**Superconducting**
- Jian/Chau 2010, Coaxial MG using bulk HTS — https://ieeexplore.ieee.org/document/5428811/
- HTS bulk in coaxial MG (2019) — https://www.researchgate.net/publication/330646621

**Eddy loss / segmentation**
- Partial magnet segmentation in PMSM (open PDF) — https://opus.lib.uts.edu.au/bitstream/10453/140691/3/Reduction%20of%20Magnet%20Eddy%20Current%20Loss%20in%20PMSM%20by%20Using%20Partial%20Magnet%20Segment%20Method.pdf
- Segmentation effect in axial-flux PM machines (multilayer-2D) — https://www.researchgate.net/publication/261461117
- Magnet loss review (2022) — https://www.researchgate.net/publication/362895574

**3D-printed electromagnetics**
- ORNL BAAM bonded NdFeB, Sci. Rep. 6:36212 — https://www.nature.com/articles/srep36212
- Huber et al., 3D print of polymer-bonded rare-earth magnets — https://arxiv.org/abs/1605.07309 (PDF https://arxiv.org/pdf/1605.07309)
- Huber et al., Topology-optimized printed magnets — https://arxiv.org/abs/1702.01968 (PDF https://arxiv.org/pdf/1702.01968)
- In-situ alignment of anisotropic printed magnets — https://arxiv.org/pdf/2201.07111
- MIT fully 3D-printed motor, Virtual & Physical Prototyping 2026 — https://www.tandfonline.com/doi/full/10.1080/17452759.2026.2613185
- AM of soft magnets review — https://www.sciencedirect.com/science/article/pii/S2542529320300791 ; MDPI Machines 13(8):679 — https://doi.org/10.3390/machines13080679

**Tools**
- magpylib (Python magnet field computation) — https://github.com/magpylib/magpylib
- MagTense (DTU magnetostatics/micromagnetics) — https://github.com/cmt-dtu-energy/MagTense
- Radia (ESRF/RadiaSoft 3-D magnetostatics, Halbach design) — https://github.com/ochubar/Radia
