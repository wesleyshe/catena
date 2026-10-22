# Log: hardware

Newest first. The format is in the root `AGENTS.md`.

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
