# AGENTS.md: simulation

Read the root `AGENTS.md` first.

This folder is a MuJoCo model of the chain:
- It floats free in zero gravity, with no tether. A floor and gravity are optional.
- You can drive it from the control window, the viewer's keys and sliders, fold-sequence CSVs, or headless.
- It's used to predict what the chain will do in the flight experiments (`../experiments/`).

`README.md` here is the user guide. This file is for agents: status, map, contracts, decisions and open issues.
History is in `LOG.md`.

## Status (2026-11-03)
- **Internally consistent, not yet checked against the real chain.**
  - `verify.py` passes all 15 sections.
  - Predictions hold for the numbers in `../hardware/spec.toml`, and some of those are placeholders.
  - An outside review found the core sound but not yet a flight prediction; see "Is the sim good enough for flight?"
    in `../experiments/findings.md`.
- **It models the AX-12A chain in the current Rhino file.** The STS3032 flight build's Rhino model is due in the coming
  weeks, and comes in through "Incorporating a new design" in `../hardware/AGENTS.md`.
- **End effectors and setups: all 6 phases built** (design and plans: `../docs/design/2026-10-2*-end-effectors*.md`).
  Anchors, suction cups, thrusters, placement, the rod, grip and nozzle columns, the stall sweep, Playback x, the
  countdown. 19 setups make the six class simulations (1a–1c, 2a–2c, 3a ×4, 4a–4b, 5a–5b, 6a); results in
  `../experiments/findings.md`. The control window's six demo buttons play one each.
- **The flight box and a whole parabola** (2026-11-03): a setup's `[flight]` puts the chain in the acrylic box sized
  from the A310 cabin (`../experiments/flight_space.md`), held by strings that let go at zero g; Fly a parabola /
  `--fly` runs the ITF's g profile. The 8.5 cm flight build is preset `feetech85`, scaled from the AX-12A chain (the
  spec stays on the AX-12A chain until the STS3032 CAD exists). Setup `1d_flight_turn`.
- **Owner: Wesley.** Wesley double-clicks `Chain Sim.command` rather than using the shell, and uses the sim to answer
  the professor's physics questions (`../experiments/findings.md`).

## Setup and commands (from this folder)
```
uv venv --python 3.11 .venv && uv pip install --python .venv/bin/python -r requirements.txt   # once
.venv/bin/python verify.py                                   # all checks + renders/, ~100 s, opens no windows
.venv/bin/python make_chain.py [--preset feetech85] [-n 12] [--slip-ring] [--gravity 9.81] [--box]   # writes chain.xml
.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless                 # logs/run_log.csv
.venv/bin/python play.py --seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air   # flight-like
.venv/bin/python play.py --setup setups/1d_flight_turn.toml --headless --fly   # one whole parabola in the flight box
.venv/bin/python play.py --setup setups/1a_turn.toml --headless   # a whole run from a setup file; flags given win
.venv/bin/mjpython play.py                                   # viewer only (macOS needs mjpython)
.venv/bin/python panel.py                                    # control window + viewer (= double-click Chain Sim.command)
.venv/bin/python export_meshes.py                            # rebuild meshes/ from ../hardware/cad/complete chain.3dm
.venv/bin/python inspect_3dm.py [FILE.3dm]                   # summary of a .3dm (default: the single module)
```
Default paths are relative to this folder, so the scripts also work when run from somewhere else.

## Folder map
| Path | What it is |
|---|---|
| `make_chain.py` | Parametric generator. The `PRESETS` table and constants at the top are what people edit. `build()` returns the XML. |
| `chain.xml` | Written by `make_chain.py` (git-ignored). Nothing else uses it: `play.py` builds the model fresh from the spec. |
| `panel_setup.toml` | Git-ignored. **The panel writes its settings (laid over the loaded setup) here every time it opens the viewer**, and starts `play.py --setup panel_setup.toml`. |
| `pole_climb_design.py` | Writes `../sequences/pole_climb.csv` and `setups/5a_pole_climb.toml` (pole size, squeeze and stretch at the top). |
| `stall_sweep.py` | Experiment A's stall length: the longest chain that stands out level from a wall at Earth, Mars, Moon and zero g (~1 min). |
| `play.py` | Runtime: `Session` (state, commands, spin, readouts), viewer loop, keyboard, `Logger`, headless mode. |
| `panel.py` | Tk control window: a separate process that drives `play.py` over a localhost socket. Its top row is the demo buttons (`setup_file.DEMOS`). Under Setup it shows the picked setup's line and `summary`; under Fold sequence, the CSV's first line and the setups that use it (`users`). |
| `viewer_link.py` | The panel's link to the viewer: starts `play.py --setup FILE --panel PORT` under mjpython, sends commands, reads readouts. |
| `setup_file.py` | Setup files: `load` (refuses unknown keys), `save` (writes TOML by hand: `ALWAYS`, then only what differs from `DEFAULTS`), `about` (a file's first `#` line, setups and CSVs alike), `summary` (what's in a setup, one line), `users` (the setups that play a sequence), `build_args`, `single_axis_spin`; `DEMOS` (the demo buttons: label, setup). |
| `setups/` | Example setups, one per class clip, named `{demo}{version}_{what}` (e.g. `5a_pole_climb`). `verify.py` checks every file in it loads and builds, and that every `DEMOS` setup exists. |
| `Chain Sim.command` | Finder double-click launcher for `panel.py`. A Terminal window stays open while it runs. |
| `verify.py` | Regression checks and renders; see "Verification". |
| `export_meshes.py` | 3dm → `meshes/`: one STL per rigid body, module frame, metres, under 20k triangles. |
| `inspect_3dm.py` | 3dm summary: units, layers, objects, tight mesh bounding boxes. |
| `meshes/` | `base_first.stl` (module 1), `base_mid.stl` (2–11), `base_last.stl` (12), `arm.stl` (1–11). Generated, but committed. `SOURCE.txt` records the CAD file name and SHA-256 they came from. |
| `renders/` | PNGs written by `verify.py` (git-ignored: they differ slightly between computers). |
| `logs/`, `screenshots/` | Run output (git-ignored). The shell default log is `logs/run_log.csv`; the panel writes `logs/run_<stamp>.csv`. |
| `.venv/` | This folder's Python 3.11 environment (uv, git-ignored). |

Fold sequences live in `../sequences/` because the robot software shares them. The CAD is in `../hardware/cad/`.

**Staying on the latest hardware:**
- `make_chain.py` reads the real chain's numbers from `../hardware/spec.toml`: module count, pitch, shell size,
  ranges, servo and masses. `original14` *is* that spec. Don't copy those numbers into code.
- `verify.py` fails if `meshes/` weren't exported from the current CAD file.
- The meshes are visual only, so stale meshes give a wrong picture but the same physics. The physics follows the spec.

## How the pieces connect
```
../hardware/cad/complete chain.3dm --export_meshes.py--> meshes/*.stl --make_chain.py--> chain.xml --> play.py
                                                                                     ^ JSON lines over 127.0.0.1
Chain Sim.command --> panel.py (Tk) -- writes panel_setup.toml, spawns `mjpython play.py --setup it --panel PORT` --+
setups/*.toml --setup_file.py--> build() args + run settings --> play.py --setup, or the panel's fields (Load / Save as)
verify.py builds models in memory with make_chain.build() and drives play.Session headless.
```

## Working with Wesley
- **Clicking, not typing.** Wesley prefers clicking to typing commands.
- **Reopen after code changes.** After editing `play.py`, `panel.py` or `make_chain.py`, tell Wesley to close the
  Chain Sim window and reopen it, because running processes keep the old code. Clicking Rebuild isn't enough: the open
  panel has already loaded the old `make_chain`.
- **Done means verified:** run `verify.py` and do a real run before saying something is done.

## Contracts (other files rely on these; keep them through any refactor)
- **Regression suite.**
  - Run `.venv/bin/python verify.py` before and after. It must end with `ALL CHECKS PASSED`.
  - Key numbers for original14:
    - Centre of mass moves ≤ 0.005 mm in the random-motion test, and |L| stays below 1e-7 of the summed body momenta.
    - Spin 20 → 20.0 deg/s.
    - An L fold → 31.9 deg/s.
    - m11_pitch 90° from rest → turned −2.2°.
  - **Log columns:** `time, com_x/y/z, L_x/y/z, L_norm, max_torque, contacts, imu_qw/qx/qy/qz, imu_wx/wy/wz`.
    Software and experiments read these, so add columns at the end rather than reordering. `max_torque` is over joint
    servos only. After the end-effector columns, a model with the flight box adds `g_x, g_y, g_z` (the g applied, in g),
    `box_gap_mm` and `string_N` (`Session.log_names`).
  - For a pure refactor, the headless run (`play.py --seq ../sequences/fold_sequence.csv --headless --log FILE`) must
    give a byte-identical log before and after.
- **Names in the model** (`make_chain.build`):
  - Bodies are `m{i}` and `m{i}_arm`.
  - Joints and actuators are `m{i}_yaw` and `m{i}_pitch`.
  - Actuator order is the CSV column order, minus `m{N}_pitch`, and minus `m1_yaw` unless ee1 is fitted. Any other
    actuators (end effectors, later) come after the joint servos; `play.joint_servos` counts these, `play.modules` the
    modules.
  - End effectors: bodies `ee1` (child of m1, on hinge `m1_yaw`, axis −Z so + turns the m12 side +, like every yaw) and
    `ee{N}` (rigid child of m{N}), with geoms `ee{k}_visual` and `ee{k}_sphere`. They're added after the chain, so the
    chain's body ids don't move. Both none → `build()` is byte-identical to before they existed.
  - An anchor adds geom `ee{k}_tip` (the grip face; collides with nothing), mocap body `ee{k}_hold` and weld
    `ee{k}_clamp` (to the hold, inactive). A suction cup adds body and geom `ee{k}_cup` (collides with nothing by
    itself; explicit contact pairs with every world geom) and adhesion actuator `ee{k}_suck`. A thruster adds sites and
    actuators `ee{k}_n0` … `n4` (push against the site's z; the site lights up while firing). `play.End` tells the
    types apart by these names. World geoms: `plate` (an attach point off the floor) and `rod`.
  - Geoms: `m{i}_visual`, which `play.py` recolours for keyboard selection, and `m{i}_sphere`, whose radius `verify.py`
    reads.
  - Sensors `com`, `com_vel` and `angmom` sit on body `m1`.
  - Sensors `imu_quat` (`framequat`) and `imu_gyro` sit on site `imu`, at the centre of the module named in `[imu]` in
    the spec. The `Logger` and `Session.imu_turn` read them.
  - `m1` must stay body id 1, with the free joint: `Session` uses `d.xpos[1]`, `subtree_com[1]` and `qvel[:6]`.
- **`play.py` API used elsewhere:**
  - `verify.py` uses `Session` (`advance`, `command`, `readouts`, `spin`, `spin_rate`, `imu_turn`, `inertia`, `turned`,
    `d`), plus `read_sequence` and `target`.
  - `make_chain.build(preset, n, slip_ring, gravity, air, ee1, ee_n, rod, flight)`. `air` takes its density from
    `[flight]` in the spec; `ee1`/`ee_n` are a type from `EE_TYPES` or a dict like `EE_DEFAULTS`; `flight` is a
    setup's `[flight]` (`box_m`, `strings`) or None. With it: world geoms `box_lid`, `box_wall_±x/±y`, `net`; sites
    `m{i}_string{k}` and `m{i}_string{k}_end`; tendons `m{i}_string{k}` (k = 0–3), limited, range set by `play.py`.
    With None, `build()` is byte-identical to before the box existed.
  - `Session(model, data, log_path, setup)`: with a setup, `reset()` (Reset pose) goes to its start pose. `Session.ends`
    maps `ee1`/`ee{N}` to `End` (kind, armed, clamped, held, load, torque, peak, pops); `grip(end, on)`,
    `nozzle(end, k, throttle)`. Per fitted end the log appends `ee{k}_held, ee{k}_load_N, ee{k}_torque_Nm, ee{k}_thrust_N`.
  - Flight: `Session.fly(path, move, hold, release_s, start_s, hyper_s, jitter)` (call `reset()` first; missing values
    come from the setup's `[flight]`), `fly_t` (None unless flying), `parabola` (`play.Parabola.at(t)` → phase, g
    vector, T), `released`, `touch`, `gap_min`, `landing`, `dip`, `flight_result()`; `strings_on(on)`, `tension()`,
    `box_gap()`. `reset()` and Stop let the strings go and put gravity back to the setup's.
  - `Session.mid` is module n//2, which is m6 of 12. `turned` tracks its heading. `Session.nj` is the joint-servo count:
    sequences write `ctrl[:nj]`. `Session.summary()` is the headless run's last line.
- **Panel ↔ viewer protocol:**
  - The panel is the socket server, on a port the OS picks. It passes `--panel PORT --log ""` to `play.py`.
  - Commands are JSON lines `{"cmd": ...}`. The panel's Play sends reset first when a take has already run (and
    Restart always does), then speed, playback, countdown (unless Countdown s is 0), spin (if "spin when Play
    starts" is ticked) and play. `play.Commands` holds the commands sent after a countdown, and the viewer holds the
    physics, until it ends; pause, stop, straighten, reset or quit during it call the take off. Pause, stop and
    straighten turn the nozzles off; resume re-applies the current row:
    play {file, move, hold}, pause, resume, restart, stop, speed {value}, playback {value}, grip {end, on},
    nozzle {end, n, on},
    gravity {g}, straighten, reset, log {on}, screenshot, physics {run}, spin {rate, axis: "x" | "y" | "z" | "all"},
    countdown {s}, fly {file, move, hold, release, start, hyper, jitter}, quit. Fly a parabola sends reset, speed,
    playback, countdown, fly.
  - Readouts are JSON lines of display strings, sent 5 times a second. The panel shows them all and reads
    `info["physics"]` to label its pause button.
- **Keep `make_chain.py`, `setup_file.py` and `panel.py` stdlib-only.** `panel.py` imports only the stdlib plus those
  two, which is what lets it run under the throwaway-bundle screenshot trick (see Gotchas).
- **Setup files:** keys are `setup_file.DEFAULTS`, plus tables `[ee1]`, `[ee{modules}]` (type + that type's
  `SETTINGS`), `[rod]` and `[flight]` (`FLIGHT_DEFAULTS`; strings default to `default_strings(modules)`). A relative `sequence` is relative to `../sequences/`. New keys go in `DEFAULTS` (and
  `build_args` if `build()` needs them); `load` refuses anything else. `[rod] wrap_m` starts the bare chain wrapped
  around the rod (`Session.wrap`): its sequence's first row must be a coil (every pitch the same, every yaw the same).
  The panel keeps the loaded setup and lays its fields over it, so keys without a field survive Save as. `save` writes
  only `ALWAYS` and non-default keys, so a setup that leaves a key out follows `DEFAULTS` (and `modules` the spec).
  `DEMOS` and `verify.py` name setups by file name: renaming one means updating both.
  - **A sequence is only the moves.** The world and end effectors a sequence was written for live in its setup, and
    each CSV's first `#` line says what it does (the panel shows it; `verify.py` checks it's there).
- **Paths:**
  - Everything resolves from the script folder (`HERE`): `meshes/`, `logs/`, `screenshots/`, `chain.xml`.
  - Sequences come from `../sequences/`, `export_meshes.py` reads `../hardware/cad/`, and `make_chain.py` reads
    `../hardware/spec.toml` when it's imported. The spec is read with `tomllib`, which is stdlib, so the stdlib-only
    rule still holds.
  - `Chain Sim.command` expects `panel.py` and `.venv/` next to it.
- **Coupled physics settings; change them together.**
  - The RK4 timestep, `armature`, `kv` and the masses: see "Physics choices" in `README.md` for the stability rule.
  - `SERVO_LAG` (5 ms) and `LOG_DT` (10 ms target updates).
  - `mj_forward` after `mj_step`, before reading sensors.
- **`README.md` is the user guide.** Keep its commands and control-window tables in sync with the code.

## Open issues / next steps
1. **Key-reset report (unresolved).** On 2026-10-17 Wesley reported that Fn + Left/Right "reset everything".
   Hypothesis: our Space (physics pause) also pauses MuJoCo's viewer, whose Left/Right then step through its history.
   To reproduce: in your own viewer, press Space, then Right, and watch `time`. Likely fix: move pause off Space or the
   yaw nudges off Left/Right; ask Wesley which keys.
2. **Dock app (offered, never answered).** A "Chain Sim.app" with no Terminal window: a bundle running
   `.venv/bin/python panel.py`, or an `osacompile` AppleScript applet that starts it in the background, logging to
   `logs/`.
3. **Placeholder parameters** (`../hardware/AGENTS.md` lists what to measure):
   - In `../hardware/spec.toml`: masses of 200 g per base and 30 g per arm, with the end modules equal to the middle
     ones. Once the modules are weighed, `module_g` takes each module's weight.
   - Also in the spec: the IMU module (`[imu]`), the box size and the net (`[flight]`). The `feetech85` masses are
     scaled estimates.
   - In `make_chain.py`: the 7 cm preset masses are rough.
   - `armature`, `kv` and `GEAR_FRICTION` are tuned estimates.
   - `ROLLING_RESISTANCE = 0.03` is tuned for settling, not measured. `KP_SATURATE = 10°`.
4. **Sliders and drift.** Slider drags are instant target jumps (one 200 s session drifted 0.68 mm; acceptable).
5. **Window placement.** The control window is always on top and covers part of the viewer's left panel.
6. **`turned` and `spin` are about the world vertical only.**
   - That's the axis pitch folds turn about, because `CHAIN_ROLL = 0` makes the pitch axes vertical.
   - For 3D, use `IMU turn` and the `imu_*` log columns.
7. **Rotor momentum isn't modelled.**
   - MuJoCo's `armature` adds the motors' inertia but not their spin momentum, about (armature/254)·q̇.
   - Over a closed shape loop it cancels.
   - For the slip-ring reaction-wheel finding it's about 3% of module 12's own momentum, so treat that result as ±3%.
8. **Experiment support.** `../experiments/README.md` lists what the flight experiments still need from the sim:
   prediction ranges over release errors, and the aircraft's own rotation.
9. **Small things from the phase 1 review** (the rest was fixed in phase 2): `joint_servos` doesn't assert the joint
   servos come first; the setup's about line follows the picker before Load, and Save as prefills it.
10. **The servo model is tuned, not identified.**
   - `kv` has no counterpart in the AX-12A (its firmware is proportional-only), and it makes joints lag their targets:
     7.6° at default timing, against 2.2° without it.
   - Net turns barely depend on it (±50% on any servo setting moves a 4-loop turn by ≤ 1.2°). Joint timing and peak
     torque do.
   - **Fix:** step-response and speed-under-load tests of one joint on the bench, then fit `kp`, `kv`, `armature` and
     friction.

11. **The flight, not yet modelled:** the plane's own rotation (~90° per parabola; the chain keeps that spin when it's
   let go), strings letting go at different times, a cloth net (the sim's net is a spring for landings, a few mm deep
   at rest), and Novespace's measured acceleration record in place of the jitter waves.

## Physics choices
The reasons behind the servo model, masses, integrator, floor, world layout, end effectors and flight options are
in "Physics choices" at the end of `README.md`. Read them before changing any of those.

## Interface details
How `spin`, the readouts, timing, screenshots and the viewer keys work is in "Interface details" at the end of
`README.md`.

## Gotchas for agents
- **macOS viewer:** it needs `mjpython`. Tk can't run inside mjpython because both need the main thread; that's why
  the panel is a separate process.
- **Don't touch other people's windows.** Check `ps` for `panel.py|play.py` first, and launch your own instance to test.
- **The viewer window (macOS)** reopens where it was (GLFW's `COCOA_FRAME_NAME` hint). Its UI size is fixed when it
  opens, and MuJoCo's Python API can't change it. See "Interface details" in `README.md`.
- **The passive viewer ignores `<visual><global azimuth/elevation>`.** `run_viewer` calls `mjv_defaultFreeCamera`
  instead, and `statistic extent` = the chain length, so the whole chain fits.
- **Screenshots (macOS):** screen tools can see the mjpython viewer but not the Tk panel; the throwaway-bundle
  workaround is in "Interface details" in `README.md`. Offscreen renders (`mujoco.Renderer`) need no permission.
- **Scripted GUI tests can press real buttons:** find the `ttk.Button` by its text, call `.invoke()`, and run with
  `python -u`.
- **End-effector actuators must override the servo defaults.** `<default><position kp kv forcerange timeconst>` sets
  every actuator's defaults, so a bare `<general>` (a nozzle) gets a 5 ms filter and a kp/kv bias that acts as a
  damper. Give them `biastype="none" dyntype="none"` (and their own `forcerange`).
- **Suction cups and MuJoCo's contact gap.** In MuJoCo 3.16 a contact's `includemargin` is its margin even with
  `gap` set, so a cup with a 10 mm margin hovered 9 mm up and pushed itself off when not sucking. Cups therefore have
  a 1 mm margin (`CUP_MARGIN`) and must be pressed on to stick: gaits aim a landing foot ~15 mm into the surface.
- **Clean up after experiments:**
  - `MUJOCO_LOG.TXT` appears when a run goes unstable; delete it afterwards.
  - Remove `__pycache__/` after test runs.

## Verification
`verify.py` has 15 sections, from compiling against the current CAD to a parabola in the flight box. Its docstring
says what each checks, and the key numbers are under Contracts.
