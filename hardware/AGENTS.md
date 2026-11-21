# AGENTS.md: hardware

Read the root `AGENTS.md` first. This folder holds the physical design of the chain: CAD, measurements, kinematics and
BOM decisions. History is in `LOG.md`.

## Status (2026-11-21)
- The 12-module chain with AX-12A servos exists and has been published (Mutis et al., SCF 2025). Its CAD is archived.
- Per the proposal, flight still needs the soft damping membrane, an IMU and onboard data logging.
- **The build is the 7 cm module with SCS0009 servos** (Wesley, 2026-11-21; CAD `cad/7cm module.3dm`):
  - Shell Ø70 mm, modules 70 mm centre to centre, so 13 shells are about 0.91 m end to end.
  - **13 modules:** m1–m11 full (a yaw and a pitch servo each), m12 with its yaw servo only, and a module with no
    servos past m1 (`ee1`, on m1's yaw servo). 23 joints.
  - **Servos ordered (Wesley):** 11 double-shaft SCS0009 for pitch, 12 single-shaft for yaw.
  - Stall torque 0.226 N·m (2.3 kg·cm), no-load 10.5 rad/s (0.1 s/60°): the figures used in the 2026-11-07 comparison.
    Check them against Feetech's datasheet at the voltage you run.
  - The pitch servo's horn faces −X (the AX-12A's faced +X), so for the same joint angle it turns the other way.
    Record it under the servo directions in `spec.toml` once they're measured.
  - The U2D2 / power hub and the battery aren't in the CAD yet.
- **Superseded: the Ø85 mm STS3032 build** (decided 2026-10-22; 7 × C036, 9 × C001 and a URT ordered 2026-11-07).
  `electronics.md` and `simulation/power_check.py` were worked out for it and need redoing for the SCS0009. The sim
  keeps it as preset `feetech85`.
- Owner: (add your name)

## Files
| Path | What it is |
|---|---|
| `cad/7cm module.3dm` | The current design: one 7 cm module (Rhino 8, mm). The source for every sim mesh: the chain repeats it every 70 mm. **Never modify.** |
| `cad/archive/` | Designs that have been replaced, kept for reference: `2026-11-21 complete chain.3dm` and `2026-11-21 Single module existing.3dm`, the 14 cm AX-12A chain. |
| `electronics.md` | The STS3032 build's electronics (superseded, see Status): controller, servo IDs, the two power designs with the sim's numbers, parts lists, wiring rules, what to check on the test servos. |
| `spec.toml` | The chain's numbers (module count, spacing, shell, ranges, servo, masses), plus the IMU's module and the flight environment (the A310's g profile and leftover g, the experiment box and its net, cabin air). **The sim and the robot software read this file**, so it's the one place to change a number. |

- The CAD tools live in `simulation/`: `inspect_3dm.py` summarises a .3dm, and `export_meshes.py` turns the module into
  sim meshes. See `simulation/AGENTS.md`.

## Incorporating a new design
New designs usually arrive as a Rhino file that someone has already made, with a request to "incorporate it". Do this:
1. **Get the file into place.** It goes in `cad/`. If Rhino has it open, ask for it to be saved and closed before you
   move anything.
2. **Inspect and compare.** Run `simulation/inspect_3dm.py` on the new file, and compare it with the design it
   replaces. Report what changed, in mm.
3. **Ask before guessing.** If joint locations, the parts that move together, or the layer names that
   `export_meshes.py` groups by have changed, ask before going further.
4. **Archive the old design.** Move it to `cad/archive/`, prefixed with the date it was retired (e.g.
   `archive/2026-11-03 complete chain.3dm`). Never delete or edit it.
5. **Re-export the meshes.** Point `SRC` in `simulation/export_meshes.py` at the new file, then re-export `meshes/`.
6. **Update the numbers.** Update `spec.toml` and the measurements and kinematics below. If the inside of the module
   changed, also update the sim's mass boxes (`CORE_BOX`, `ARM_BOX` in `simulation/make_chain.py`).
7. **Verify.** Run `simulation/verify.py`, and do a real launch of the control window.
8. **Mark the findings.** Findings in `experiments/findings.md` were measured on the old design. Say so next to them,
   or re-run them.
9. **Log it.** Add entries to this `LOG.md` and `simulation/LOG.md`. If a shared fact changed (joint names, ranges,
   module count), also log it in the root `LOG.md`.

## Reading the CAD
- `7cm module.3dm` holds one module, joints at zero, chain along +Z. Its centre, where the two lines in `V6::axis` (yaw
  along Z, pitch along X) cross, is at x = −443.682, y = −504.497, z = −10.72; the shell sphere's centre is within
  0.003 mm of it. `export_meshes.py` reads the centre from those lines.
- Layers: `V6::shell`, `V6::C-handle` (the arm), `V6::servos` (both servos, 9 parts each), `V6::body` (the two plates
  joining the servos), `V6::axis`. `Layer 01` is empty.
- Every object has a stored render mesh, so Rhino isn't needed to read the file. rhino3dm's `GetBoundingBox()` is
  loose (it's built from control points), so measure with mesh bounds instead.

## Measurements
All in the module frame: the origin is where the yaw and pitch axes cross, which is the sphere centre. From the CAD
(mesh bounds and ray tests, 2026-11-21).
- **Shell:** outer Ø70.0 mm, 2.5 mm wall, one piece.
  - A 13 mm slot runs over the top in the Y–Z plane and closes 128.2° from +Z on both sides.
  - The bottom opening is Ø15 mm.
- **Servos (SCS0009):** bodies 12.1 × 23.25 mm, 25.5 mm tall without the spline; 32.6 mm across the mounting ears.
  - Yaw servo: z −31.6…−6.1, spline (Ø4 mm, 3.25 mm long) facing −Z on the axis, its end at z = −34.55.
  - Pitch servo: lies along X, x −12.9…+16.6, spline facing −X on the axis, its end at x = −15.85.
- **Body:** two 12.1 × 3.7 × 29 mm plates on the servos' ears (y −21.9…−18.2 and +7.1…+10.7). Nothing in the CAD yet
  fixes them to the shell.
- **Fit to check:** the yaw servo's two ear corners on the −Y side (x ±5.7, y −21.8, z −24.4) reach 33.2 mm from the
  centre, 0.76 mm into the shell wall (its inside face is at 32.47 mm there). The −Y plate clears it by 0.22 mm.
- **Arm (C-handle):** 40 mm wide (pitch pivots at x = ±20), its stem 10 × 10 mm where it passes the shell slot, the hub
  top at +38.3 mm.
- **Pitch (module spacing): 70 mm** (Wesley). The next module's yaw spline then sits 2.85 mm into the hub, and its
  servo case clears the hub top by 0.1 mm. Neighbouring shells meet only on the chain axis, where the slot and the
  bottom opening are.
- **Pitch range:** the stem hits the end of the slot at 119.6°; shells two modules apart would touch at 120.0°.
- **Volumes from CAD:** shell 30.7 cm³ (38 g in PLA), body plates 2.4 cm³ (3 g), C-handle 4.3 cm³ (5 g; its mesh isn't
  closed, so roughly).

## Kinematics (confirmed by Wesley)
- **Two rigid bodies per module:**
  - The base: shell, body and both servos.
  - The arm: the C-handle.
- **Joints:**
  - `m{i}_pitch`: base i → arm i, about the module's +X axis through its centre.
  - `m{i+1}_yaw`: arm i → base i+1, about +Z. The hub takes the next module's yaw spline.
- **End modules (2026-11-21):**
  - m1 is a full module. Its yaw servo turns `ee1`, a module with no servos (shell and a fixed C-handle) past it.
  - m12 has a yaw servo but no pitch servo and no C-handle.
  - Where the U2D2 and power hub go isn't drawn yet.

## Numbers the sim still guesses (please measure)
- **Masses** (in `spec.toml`):
  - The spec says 90 g per base and 6 g per arm (2026-11-21 placeholders: Wesley expects about 100 g a module; the
    printed parts are about 46 g in PLA, see Measurements). End modules are assumed to weigh the same as middle ones.
  - Weigh each whole module in flight configuration, battery included, and list them in `module_g`. Weigh one arm on
    its own for `arm_g`.
  - It's where the weight sits that changes the predicted turn (a battery at the ends: 9%), not the total.
- **Placeholders for the flight** (in `spec.toml`): which module carries the IMU (`[imu]`); the experiment box's size
  (`box_mm`, 2.0 × 1.3 × 1.9 m: fits one side of the A310 cabin with the aisle clear, built on board from parts,
  `../experiments/flight_space.md`), the net's height and sag, and the plane's pitch (`pitch_deg`) (`[flight]`).
- **Gearbox behaviour** (in `simulation/make_chain.py`): friction (7% of stall torque), motor inertia and braking are
  tuned estimates. There are no measured SCS0009 values.

## Miniaturisation (decided 2026-10-17)
- **Goals:**
  - No slip ring: wired, so yaw is limited by cable twist.
  - As small as possible.
  - A layout as close to the 14 cm module as possible: 2 servos per module, the yaw servo under the pitch servo, both
    axes through the sphere centre, and the same end modules.
- **The limit:** the yaw servo's far bottom corner sits at 65.1 mm against a 67.5 mm inner radius, leaving 2.4 mm.
- **Estimated smallest shells:** SCS0009 about 73 mm, STS3032 about 79 mm, XL330/XC330 about 96 mm.
  - So 5.5 cm modules can't fit two servos, and the 7 cm sim presets (`dyna7`, `feetech7`) are optimistic what-ifs.
  - These estimates come from reseller dimensions plus assumed shaft offsets. No STEP files were checked.
- **Outcome:** the STS3032 build at about 80 mm (2026-10-22), Ø85 mm from 2026-11-03; then the 7 cm SCS0009 module
  (2026-11-21, `cad/7cm module.3dm`), 3 mm under the 73 mm estimate above with no clearance left: the yaw servo's
  ears reach into the wall (Measurements).
- **Before sizing a mini module:**
  - Confirm the servo choice, and whether about 7.3–8 cm is acceptable.
  - Going smaller needs a layout change, so ask first.

## Checks
- There's no automated test in this folder.
- Report measurements with units and how you got them (calipers, CAD, scale).
- If geometry the sim uses changes, the sim meshes need re-exporting and `verify.py` re-running (see
  `simulation/AGENTS.md`). Log that in both areas.
