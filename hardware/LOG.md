# Log: hardware

Newest first. The format is in the root `AGENTS.md`.

## 2026-11-21 · Wesley (Claude Code)
- **New design: the 7 cm SCS0009 module**, `cad/7cm module.3dm` (Wesley's Rhino file, re-saved 13:11 with a `V6::axis`
  layer), brought in by "Incorporating a new design". The 14 cm chain's two files moved to `cad/archive/`, prefixed
  `2026-11-21`.
- `spec.toml`: pitch 70 mm, shell Ø70 mm, pitch ±119° (the C-handle's stem hits the slot's end at 119.6°), yaw ±150°,
  SCS0009 at 0.226 N·m and 10.5 rad/s, masses 90 / 90 / 6 g (placeholders). `AGENTS.md`: status, files, reading the
  CAD, measurements, kinematics.
- **The build (Wesley):** m1–m11 full, m12 yaw only, and a module with no servos past m1 (`ee1`, on m1's yaw servo):
  13 shells, 23 joints. Servos ordered: 11 double-shaft SCS0009 for pitch, 12 single-shaft for yaw. The Ø85 mm STS3032
  build is superseded, and `electronics.md` and `simulation/power_check.py` need redoing for the SCS0009.
- **To check in the CAD:** the yaw servo's two −Y ear corners reach 33.23 mm from the centre, 0.76 mm into the shell
  wall (its inside face is at 32.47 mm there). The pitch servo's horn faces −X; the AX-12A's faced +X.
- **Checked:** the CAD measured with mesh bounds and ray tests; the axis lines cross within 0.004 mm of the centre I had
  measured; `simulation/verify.py` ALL CHECKS PASSED.

## 2026-11-07 · Wesley (Claude Code)
- New `electronics.md`: the STS3032 build's controller (one FE-URT-1, Feetech's Python SDK), bus and servo IDs, the
  first build's order (7 × C036, 9 × C001 and the URT, for 8 modules plus `ee1`), the two power designs with the sim's
  numbers, parts lists, wiring rules and what to check on the test servos. Worked out with Wesley 2026-10-17 to 11-07.
- `AGENTS.md` Status: the "converter, junction board and capacitor per module" line became the undecided power choice
  (converters or a thick 6 V cable) with a pointer to `electronics.md`; the order is recorded. Files table: the new file.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED (no spec numbers changed).

## 2026-11-03 · Wesley (Claude Code)
- `AGENTS.md`, Miniaturisation: the 2026-10-22 "about 80 mm" outcome now says it became Ø85 mm on 2026-11-03.
- `spec.toml`: a comment only. The ITF it cites is Novespace's, so it isn't in git; ask Wesley for the PDF.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED (no numbers changed).

## 2026-11-03 · Wesley (Claude Code)
- `spec.toml`: `net_height_mm` 200 → 300 (Wesley: a buffer under the net so a falling chain can't reach the floor; the
  sim's landings sink at most 115 mm).
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED.

## 2026-11-03 · Wesley (Claude Code)
- `spec.toml` `[flight]`: `box_mm` 2500 → 2000 along the cabin (Wesley: "a reasonable size that can be carried onto the
  plane"); it still needs building on board from parts, since it's bigger than the door (`experiments/flight_space.md`).
  New `pitch_deg` = 45: the plane's nose-up as zero g begins (typical; check Novespace's attitude data).
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED.

## 2026-11-03 · Wesley (Claude Code)
- **The flight build's module is Ø85 mm, 0.5 mm apart** (Wesley). Until its CAD exists the sim scales the AX-12A module
  to it (preset `feetech85`); the spec's `[chain]` stays on the AX-12A chain.
- **`spec.toml` `[flight]`, from the Novespace A310 interface document** (`docs/ITF-2025-01-Interfaces-EN.pdf`):
  `parabola_s` 20 → 22 s; new `hypergravity_g` (1.8), `hypergravity_s` (23), `jitter_g` (±0.015 / 0.015 / 0.02 g);
  `cage_mm` replaced by `box_mm` = 2500 × 1300 × 1900 (along the cabin, across, high: the largest box that fits one side
  of the aisle, worked out in `experiments/flight_space.md`), plus placeholder `net_height_mm` (200) and `net_sag_mm` (20).
- **For the other areas:** the sim reads all of these (see `simulation/LOG.md`); nothing in software yet.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED.

## 2026-10-22 · Wesley (Claude Code)
- **Recorded the flight build decision:** STS3032 servos, ~80 mm shells, ~0.96 m chain, with the BOM details in
  `AGENTS.md`. Its Rhino model is due in the coming weeks; the spec stays on the AX-12A chain until then.
- **`spec.toml` gained:**
  - `module_g`, for weighed modules (commented out until they're weighed)
  - `[imu] module` (placeholder: 1)
  - `[flight]`: parabola length, cage size (placeholder: 2000 × 1000 × 1000 mm) and cabin air density

## 2026-10-22 · Wesley (Claude Code)
- **Added `spec.toml`:** module count, spacing, shell, ranges, servo and masses, with the same values the sim used
  before. The sim reads it, and the software should too.
- Changing a number there now reaches both; no code edits needed.

## 2026-10-22 · Wesley (Claude Code)
- Moved the two `.3dm` files into `hardware/cad/`. Their contents are unchanged.
- Moved the measurements, kinematics and miniaturisation notes here from the old `CLAUDE.md`.
- Added the "Incorporating a new design" procedure. Replaced designs go to `cad/archive/`.

## 2026-10-17 · Wesley (Claude Code)
- **Miniaturisation goals set:** no slip ring, as small as possible, layout as close to the 14 cm module as possible.
- **Size limit found:** the yaw servo's bottom corner leaves 2.4 mm clearance.
  - Smallest shells: SCS0009 ~73 mm, STS3032 ~79 mm, XL330 ~96 mm.
  - So 5.5 cm and 7 cm modules can't fit two servos in this layout.
- **Measured the module from the CAD:** shell, servos, yoke, 140 mm pitch. Wesley confirmed the kinematics.
