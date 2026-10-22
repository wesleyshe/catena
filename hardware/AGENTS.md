# AGENTS.md: hardware

Read the root `AGENTS.md` first. This folder holds the physical design of the chain: CAD, measurements, kinematics and
BOM decisions. History is in `LOG.md`.

## Status (2026-10-22)
- The 12-module chain with AX-12A servos exists and has been published (Mutis et al., SCF 2025).
- Per the proposal, flight still needs the soft damping membrane, an IMU and onboard data logging.
- A smaller module (longer chain, lower mass) is being considered; see "Miniaturisation".
- Owner: (add your name)

## Files
| Path | What it is |
|---|---|
| `cad/complete chain.3dm` | The current design: the whole 12-module chain (Rhino 8, mm). The source for every sim mesh. **Never modify.** |
| `cad/Single module existing.3dm` | One module of the current design. **Never modify.** |
| `cad/archive/` | Designs that have been replaced, kept for reference (create it when the first one is retired). |

- The single-module file matches module 2 of the chain to within 0.000 mm.
- It adds a Ø8.4 × 3 mm plug in the hub hole; ignore the plug.
- The CAD tools live in `simulation/`: `inspect_3dm.py` summarises a .3dm, and `export_meshes.py` turns the chain into
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
6. **Update the numbers.** Update the measurements and kinematics below, and the matching numbers in
   `simulation/make_chain.py` (pitch, `R_SHELL`, mass boxes, ranges, masses).
7. **Verify.** Run `simulation/verify.py`, and do a real launch of the control window.
8. **Mark the findings.** Findings in `experiments/findings.md` were measured on the old design. Say so next to them,
   or re-run them.
9. **Log it.** Add entries to this `LOG.md` and `simulation/LOG.md`. If a shared fact changed (joint names, ranges,
   module count), also log it in the root `LOG.md`.

## Reading the CAD
- In `complete chain.3dm` the modules lie along +Z at x = −400.70, y = −399.53, with centres at z = 52 + 140·k (mm).
- Every Brep, Extrusion and Mesh has a stored render mesh, so Rhino isn't needed to read the files. Curves and points
  are reference geometry: axis lines and Ref Geom points.
- rhino3dm's `GetBoundingBox()` is loose because it's built from control points, so use mesh bounds instead. For
  example, the shell measures 137.5 mm in Y, not 162.

## Measurements
All in the module frame: the origin is where the yaw and pitch axes cross, which is the sphere centre.
- **Shell:** outer Ø139.0 mm, 2.0 mm wall, split into ±X halves at x = 0.
  - A 20 mm slot runs over the top in the Y–Z plane and closes 130° from +Z.
  - The hub shaft hits the end of the slot at about 124°, which sets the ±124° pitch limit.
  - The bottom opening is Ø40 mm.
- **Servos (AX-12A):** 32 × 50 × 38 mm. The often-quoted 40 mm includes the horn, which isn't modelled.
  - Yaw servo: z −50…−12, horn facing −Z on the axis.
  - Pitch servo: z −11.5…+38.5, horn facing +X at x = +19.
- **Bracket:** one solid holding both servos. Pins lock it to both shell halves.
- **Arm (yoke):** 64 mm wide, with Ø24 mm side discs at x = ±21.
  - The hub flange's top is at +88 mm.
  - The flange and discs have the AX-12A horn's 16 mm bolt circle.
- **Pitch (module spacing): 140 mm**, confirmed by Wesley and the chain file. It's 88 mm to the hub top plus 52 mm to
  the next yaw horn face. Neighbouring shells are 1 mm apart. The original "15 cm" figure was wrong.
- **Mass from CAD (solid PLA):** base about 0.28 kg with servos; arm 24 g.

## Kinematics (confirmed by Wesley)
- **Two rigid bodies per module:**
  - The base: shell, bracket and both servos.
  - The arm: yoke and hub.
  - There's no separate yaw stage.
- **Joints:**
  - `m{i}_pitch`: base i → arm i, about the module's +X axis through its centre.
  - `m{i+1}_yaw`: arm i → base i+1, about +Z. The hub bolts to the next module's downward-facing yaw horn.
- **End modules:**
  - Module 1 has a pitch servo but no yaw servo.
  - Module 12 has a yaw servo but no pitch servo and no arm.
  - Both carry the U2D2 and power hub.

## Numbers the sim still guesses (please measure)
These are placeholders in `simulation/make_chain.py`. Measure them and log the values here and in `simulation/LOG.md`.
- **Masses:** the model uses 0.20 kg per base and 0.03 kg per arm; the CAD suggests about 0.28 kg and 24 g. End
  modules are assumed to weigh the same as middle ones.
- **Gearbox behaviour:** friction (7% of stall torque), motor inertia and braking are tuned estimates. There are no
  measured AX-12A values.

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
- **Before sizing a mini module:**
  - Confirm the servo choice, and whether about 7.3–8 cm is acceptable.
  - Going smaller needs a layout change, so ask first.

## Checks
- There's no automated test in this folder.
- Report measurements with units and how you got them (calipers, CAD, scale).
- If geometry the sim uses changes, the sim meshes need re-exporting and `verify.py` re-running (see
  `simulation/AGENTS.md`). Log that in both areas.
