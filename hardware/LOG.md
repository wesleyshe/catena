# Log: hardware

Newest first. The format is in the root `AGENTS.md`.

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
