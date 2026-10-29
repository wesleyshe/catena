# Log: simulation

Newest first. The format is in the root `AGENTS.md`.

## 2026-10-29 · Wesley (Claude Code)
- **Fixes from a fresh review of phases 2–6:**
  - A second take (Play again, or Restart) now starts from the start pose. Before, it carried on from where the last
    take ended: 5a crawled off the end of the rod, 4a tumbled.
  - Pause, Stop and Straighten turn the nozzles off (a stopped burst kept accelerating the chain); Resume re-applies
    the current row.
  - With both ends attached, the second end now holds whatever it lands on, as the design and the panel say.
  - Left for later (minor): the panel's end names go stale if Modules changes without Rebuild; nozzle tick boxes
    don't follow the sequence; `rod_m` isn't range-checked; anchors latch tilted up to ~60°; the viewer's default
    camera crops some clips; a thruster's readout says "thruster, thruster"; `turned` means little when anchored.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; the real control window by scripted buttons: Load 5a, Play
  ×8 to the end, Play again and Restart both start afresh (identical drift at the same point, 308 mm).

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 6: wrap-up.** `README.md` covers the end effectors, every setup (grouped as the six class
  simulations), Playback x and the new log columns. `../experiments/findings.md` has "The class simulations" (each
  clip's measured result, the stall length, the suction-cup tipping requirement, the side-nozzle roll), and
  `../experiments/README.md` moves the held end and the anchored arm out of "Not in the sim yet".
- **Checked:** every one of the 18 setups headless gives the result its description says; filmstrips of 11 of them
  rendered offscreen and looked at (in `renders/clips/`, not in git).

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 5: the rod crawl.** `rod_crawl.csv` and setup `5a_rod_crawl`: clamps on both ends crawl along
  a Ø35 mm rod with the same arch gait as the walk (landing exactly on the rod; a clamp grabs from 15 mm and pulls in).
  It moves 1.115 m along the rod in 19 s (two steps), one clamp always holding, peak clamp load 3.7 N.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; 5a headless.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 4: thrusters.** Five nozzle sites per thruster end (n0 out along the axis, n1–n4 along the
  end's +x, +y, −x, −y), each a site actuator pushing against its exhaust, lit while firing. Sequences
  `thrust_straight/U/Z/L.csv` and setups 3a ×4: a half-throttle burst of both n0, then coasting.
  - Results: straight doesn't move; the U glides 0.93 m and turns 1°; the Z spins at 8.2°/s and its centre of mass
    moves 0.16 mm; the L moves 0.66 m and turns 43°.
  - Finding: every nozzle's push goes through the chain's axis, but each module's centre of mass sits ~4.8 mm off that
    axis (the servo block isn't centred in the shell). So n2/n4 turn it with no roll, while n1/n3 also roll it,
    ~12.8°/s after 2 s of 0.1 N, because the chain is so easy to roll about its length.
  - 3b (glide, then turn) skipped: optional in the design, and 3a covers the thrusters.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; the four 3a setups headless.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 3: suction cups.** A cup body at the end's tip with an adhesion actuator (the pull limit),
  touching only fixed things through explicit contact pairs; held = sucking and on a surface; pull = suction minus the
  surface's push; pops counted. Setups 4a (inchworm walk, `inchworm_walk.csv`) and 4b (lets go too early,
  `walk_let_go.csv`).
  - Finding: the default 40 mm, 20 N cup peels under the stepping moment and the chain pivots off; 80 mm, 60 N walks
    (1.10 m in two steps, 19 s). The pull it feels is only ~1 N: what sizes the cup is the tipping moment, about
    pull × radius ≈ 2.4 N·m.
  - MuJoCo 3.16 doesn't let `gap` switch off a contact's push inside its margin, so cups have a 1 mm margin and gaits
    press each landing foot ~15 mm into the floor (see Gotchas in `AGENTS.md`).
- **Checked:** `verify.py` ALL CHECKS PASSED, 11 sections; 4a walks two steps with no pops, 4b floats away.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 2: placement and the anchor** (plan `../docs/design/2026-10-29-end-effectors-phases2-6-plan.md`):
  - Setups can start an end attached (`attach`: a point and facing, or along the rod), and start the chain in its
    sequence's first row. `[rod]` adds a fixed rod.
  - The anchor clamps (a weld to a mocap body), grabs on touch when told to hold (its tip within 15 mm, then pulls in
    softly over 0.2 s), and lets go; load and torque are smoothed and logged (4 columns per fitted end).
  - Sequences can carry `ee{k}_grip` columns. New: `arm_reach.csv`, `arm_let_go.csv`, `stand_out.csv`; setups 2a,
    2b ×4, 2c. `stall_sweep.py`: the longest chain that stands out level is 3 modules on Earth, 4 on Mars, 6 on the
    Moon, all 12 in zero g (hand estimate 2 / 4 / 7).
  - Wesley asked for a speed multiplier for watching: **Playback x** (panel), `--playback`, setup key `playback`. It
    runs more physics steps per frame, so the physics doesn't change; the readout says if the computer can't keep up.
  - The control window lays its fields over the loaded setup and starts the viewer from `panel_setup.toml` (the
    review's I1), and has per-type end-effector settings, Hold / Let go, nozzle toggles and a rod tick box. Two
    columns now.
  - The phase 1 review's minors in `setup_file.py` are fixed (relative paths, strict `toml()`, `isfile`, file names
    in errors).
- **Checked:** `verify.py` ALL CHECKS PASSED, 10 sections; with both ends none, `build()` and the headless logs are
  byte-identical to `fbe880f`; the 1a setup log equals the plain-flags run; every new setup headless; the real
  control window by scripted buttons (Load 2a, Play ×4, Let go, Hold, Save as keeps the placement). Not looked at by eye.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors and setups, phase 1** (design `../docs/design/2026-10-28-end-effectors.md`, plan next to it):
  - `build(..., ee1, ee_n)`: an optional module past m1, on the `m1_yaw` twist servo, and one past module N, rigid.
    Each is none, anchor, suction or thruster; so far they differ only in mass (placeholders in `make_chain.py`) and
    colour.
  - `play.py` counts joint servos and modules instead of assuming every actuator is a joint (`modules`,
    `joint_servos`, `Session.nj`). Headless runs end with a summary line.
  - Setup files: `setup_file.py`, `play.py --setup`, and five example setups in `setups/` (1a, 1b ×2, 1c, 6). The
    control window has a Setup box (Load, Save as), an End effectors box, and "spin when Play starts".
  - 1a/1b use flight timing (move 1 s, hold 0.25 s), not ×4, because the release-spin numbers in `findings.md` were
    measured at that timing.
  - `AGENTS.md` was over 250 lines: "Physics choices" moved to the end of `README.md`.
  - From a fresh review: setups refuse `move_s <= 0`, `hold_s < 0` and `speed <= 0` (they hung a headless run or
    crashed the viewer), and verify's random motion moves only joint servos. The rest is open issue 9 in `AGENTS.md`.
- **Checked:**
  - `verify.py`: ALL CHECKS PASSED, now 9 sections; sections 1–8 print exactly what they did before.
  - Both ends none: `build()` output and the headless logs (fold sequence; flight-like with spin, air, cage) are
    byte-identical to `fbe880f`.
  - Setups headless: 1a turned −30.3°, 1b −15.2° / 0.0°, 1c IMU turn 63.4°, 6 back to +20.0°/s, matching `findings.md`.
  - The real control window, with scripted button presses: Load 6_skater → Play (spin +20.0°/s), ee1 thruster →
    Rebuild (`chain.xml` has `ee1` and `m1_yaw`), Save as → Load gave the same fields. Not yet looked at by eye.

## 2026-10-22 · Wesley (Claude Code)
- **Fixes from the outside review:**
  - Spin now changes one axis at a time; it used to reset the other two. Stop spin zeroes all three.
  - The servo force limit now includes the gear friction, so the net output at stall is the rated stall torque. It was
    7% low. The key numbers in `verify.py` didn't move.
  - Docs: `kv` brakes during all unsaturated motion, not only near the target. Rotor momentum is about 3% for the
    reaction-wheel finding.
- **New flight options:**
  - **IMU output:** sensors `imu_quat` and `imu_gyro` on the module in `[imu]`, 7 new log columns, and an `IMU turn`
    readout.
  - **Spin about x, y or z:** the panel's axis choice, or `--spin X Y Z`, for leftover spin at release.
  - **Air drag and the net cage:** `--air` and `--cage`, or the panel tick boxes. Their numbers are in `[flight]`.
  - **`--speed`:** the sequence readout shows the sequence's length and flags anything longer than one parabola.
  - **`module_g`** in the spec, for weighed modules.
- **Checked:**
  - `verify.py` ALL CHECKS PASSED (now 8 sections).
  - Real launch through the real buttons: ticked Air drag and Net cage, Rebuild, Play, Spin about the across axis. 3 of
    4 runs completed every step; one early run stopped after Rebuild with no error and didn't happen again.

## 2026-10-22 · Wesley (Claude Code)
- `renders/` is no longer in git: `verify.py` rewrites it, and the PNGs differ between computers, which caused binary
  merge conflicts in a team rehearsal. The README image is now `../docs/images/module.png`.

## 2026-10-22 · Wesley (Claude Code)
- **original14 now comes from `../hardware/spec.toml`:** `make_chain.py` reads it for servo, pitch, masses, shell
  radius, ranges and module count. The panel's module default comes from it too.
- **Stale-mesh check:** `export_meshes.py` writes `meshes/SOURCE.txt` (CAD file name + SHA-256), and `verify.py`
  section 1 fails if the CAD has changed since.
- **`play.py` without `--model` builds the model fresh from the spec.** `chain.xml` is now git-ignored; the panel
  still writes it.
- Checked:
  - `verify.py` ALL CHECKS PASSED.
  - The XML for all 3 presets × slip ring is byte-identical to before, and so is the headless log.
  - Re-exported STLs unchanged.
  - Real launch: Play, 0 contacts.

## 2026-10-22 · Wesley (Claude Code)
- **Moved the sim into `simulation/`, with its own `.venv`.**
  - Sequences now come from `../sequences/` and the CAD from `../hardware/cad/`.
  - The defaults for `play.py --model/--log`, `make_chain.py -o` and `export_meshes.py` now resolve from this folder.
  - Code behaviour is unchanged.
  - `CLAUDE.md` became `AGENTS.md`.
- **Checked:**
  - `verify.py`: ALL CHECKS PASSED.
  - The headless `fold_sequence.csv` log, `chain.xml` and all four exported STLs are byte-identical to before.

## 2026-10-21 · Wesley (Claude Code)
- **Added Spin / Stop spin** to the control window, plus `spin` and `turned` readouts.
- **Checked:**
  - Tested Spin → Play → Stop spin through the real buttons.
  - Added the spin checks to `verify.py`.

## 2026-10-20 · Wesley (Claude Code)
- **Anti-wobble servo model:** gearbox friction, motor inertia (`armature`) and near-target braking (`kv`).
  - A 60° step now overshoots 1° instead of 9.5°.
- **Added `sequences/reorient_loop.csv`:** the zero-angular-momentum "falling cat" turn.

## 2026-10-18 · Wesley (Claude Code)
- **Removed the Home (straighten) key** after a report that Fn+arrows reset everything. Fn+Right is still
  unexplained; see open issue 1 in `AGENTS.md`.

## 2026-10-17 · Wesley (Claude Code)
- **First version:**
  - Meshes exported from `complete chain.3dm`.
  - `make_chain.py` with 3 presets.
  - `play.py` (viewer, sequences, logging) and `verify.py`.
  - The control window (`panel.py`, `Chain Sim.command`).
- **Checked:** tested clicking Play in the real GUI.
