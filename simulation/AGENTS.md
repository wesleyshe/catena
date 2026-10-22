# AGENTS.md: simulation

Read the root `AGENTS.md` first.

This folder is a MuJoCo model of the chain:
- It floats free in zero gravity, with no tether. A floor and gravity are optional.
- You can drive it from the control window, the viewer's keys and sliders, fold-sequence CSVs, or headless.
- It's used to predict what the chain will do in the flight experiments (`../experiments/`).

`README.md` here is the user guide. This file is for agents: status, map, contracts, decisions and open issues.
History is in `LOG.md`.

## Status (2026-10-22)
- **Everything works.** `verify.py` passes all 7 sections.
- **Owner: Wesley.** Wesley double-clicks `Chain Sim.command` rather than using the shell, and uses the sim to answer
  the professor's physics questions (`../experiments/findings.md`).

## Setup and commands (from this folder)
```
uv venv --python 3.11 .venv && uv pip install --python .venv/bin/python -r requirements.txt   # once
.venv/bin/python verify.py                                   # all checks + renders/, ~60 s, opens no windows
.venv/bin/python make_chain.py [--preset dyna7] [-n 12] [--slip-ring] [--gravity 9.81]   # writes chain.xml
.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless                 # logs/run_log.csv
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
| `chain.xml` | Generated and git-ignored. **`panel.py` rewrites it from its Model settings every time it starts or rebuilds.** `play.py` without `--model` doesn't use it: it builds the model fresh from the spec. |
| `play.py` | Runtime: `Session` (state, commands, spin, readouts), viewer loop, keyboard, `Logger`, headless mode. |
| `panel.py` | Tk control window: a separate process that drives `play.py` over a localhost socket. |
| `Chain Sim.command` | Finder double-click launcher for `panel.py`. A Terminal window stays open while it runs. |
| `verify.py` | Regression checks and renders; see "Verification". |
| `export_meshes.py` | 3dm → `meshes/`: one STL per rigid body, module frame, metres, under 20k triangles. |
| `inspect_3dm.py` | 3dm summary: units, layers, objects, tight mesh bounding boxes. |
| `meshes/` | `base_first.stl` (module 1), `base_mid.stl` (2–11), `base_last.stl` (12), `arm.stl` (1–11). Generated, but committed. `SOURCE.txt` records the CAD file name and SHA-256 they came from. |
| `renders/` | PNGs from `verify.py`. |
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
Chain Sim.command --> panel.py (Tk) -- writes chain.xml, spawns `mjpython play.py --panel PORT` --+
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
  - For a pure refactor, the headless run (`play.py --seq ../sequences/fold_sequence.csv --headless --log FILE`) must
    give a byte-identical log before and after.
- **Names in the model** (`make_chain.build`):
  - Bodies are `m{i}` and `m{i}_arm`.
  - Joints and actuators are `m{i}_yaw` and `m{i}_pitch`.
  - Actuator order is the CSV column order, minus `m1_yaw` and `m{N}_pitch`.
  - Geoms: `m{i}_visual`, which `play.py` recolours for keyboard selection, and `m{i}_sphere`, whose radius `verify.py`
    reads.
  - Sensors `com`, `com_vel` and `angmom` sit on body `m1`.
  - `m1` must stay body id 1, with the free joint: `Session` uses `d.xpos[1]`, `subtree_com[1]` and `qvel[:6]`.
- **`play.py` API used elsewhere:**
  - `verify.py` uses `Session` (`advance`, `command`, `readouts`, `spin_rate`, `inertia`, `turned`, `d`), plus
    `read_sequence` and `target`.
  - `Session.mid` is module (n+2)//4, which is m6 of 12. `turned` tracks its heading.
- **Panel ↔ viewer protocol:**
  - The panel is the socket server, on a port the OS picks. It passes `--panel PORT --log ""` to `play.py`.
  - Commands are JSON lines `{"cmd": ...}`: play {file, move, hold}, pause, resume, restart, stop, speed {value},
    gravity {g}, straighten, reset, log {on}, screenshot, physics {run}, spin {rate}, quit.
  - Readouts are JSON lines of display strings, sent 5 times a second. The panel shows them all and reads
    `info["physics"]` to label its pause button.
- **Keep `make_chain.py` and `panel.py` stdlib-only.** `panel.py` imports only the stdlib plus `make_chain`, which
  is what lets it run under the throwaway-bundle screenshot trick (see Gotchas).
- **Paths:**
  - Everything resolves from the script folder (`HERE`): `meshes/`, `logs/`, `screenshots/`, `chain.xml`.
  - Sequences come from `../sequences/`, `export_meshes.py` reads `../hardware/cad/`, and `make_chain.py` reads
    `../hardware/spec.toml` when it's imported. The spec is read with `tomllib`, which is stdlib, so the stdlib-only
    rule still holds.
  - `Chain Sim.command` expects `panel.py` and `.venv/` next to it.
- **Coupled physics settings; change them together.**
  - The RK4 timestep, `armature`, `kv` and the masses: see Physics for the stability rule.
  - `SERVO_LAG` (5 ms) and `LOG_DT` (10 ms target updates).
  - `mj_forward` after `mj_step`, before reading sensors.
- **`README.md` is the user guide.** Keep its commands and control-window tables in sync with the code.

## Open issues / next steps
1. **Key-reset report (unresolved).**
   - On 2026-10-17 Wesley said "fn and left right [arrows] now reset everything" and asked for it to be removed.
   - Fn+Left is Home, which was "straighten"; that binding was removed on 2026-10-18.
   - Fn+Right is End, which was never bound. Not reproduced yet.
   - **Hypothesis:** MuJoCo's viewer has built-in keys: Space = Play/Pause, Left/Right = step through history when
     paused, and others. Our Space (physics pause) probably also toggles the viewer's own pause, so our Left/Right yaw
     nudges then scrub its history and load an old state, which looks like a reset.
   - **To reproduce:** start a separate viewer (`.venv/bin/mjpython play.py --log ""`), click that window, press Space,
     then Right, and watch `time`.
   - **Likely fix:** move the physics pause off Space (keep the panel button) and/or move the yaw nudges off
     Left/Right. Ask Wesley which keys to use.
2. **Dock app (offered, never answered).** A "Chain Sim.app" with no Terminal window: a bundle running
   `.venv/bin/python panel.py`, or an `osacompile` AppleScript applet that starts it in the background, logging to
   `logs/`.
3. **Placeholder parameters** (`../hardware/AGENTS.md` lists what to measure):
   - In `../hardware/spec.toml`: masses of 200 g per base and 30 g per arm, with the end modules equal to the middle
     ones.
   - In `make_chain.py`: the 7 cm preset masses are rough.
   - `armature`, `kv` and `GEAR_FRICTION` are tuned estimates.
   - `ROLLING_RESISTANCE = 0.03` is tuned for settling, not measured. `KP_SATURATE = 10°`.
4. **Sliders and drift.** Slider drags are instant target jumps. One 200 s session drifted 0.68 mm, against 0.005 mm in
   `verify.py`. That's acceptable.
5. **Window placement.** The control window is always on top and covers part of the viewer's left panel.
6. **Spin and turned are about the world vertical only.** That's the axis pitch folds turn about, because
   `CHAIN_ROLL = 0` makes the pitch axes vertical. There's no Spin about other axes, and `turned` ignores twists about
   the chain's own axis.
7. **Rotor momentum isn't modelled.** MuJoCo's `armature` adds the motors' inertia but not their spin momentum, which
   is about (armature/254)·q̇, roughly 0.006% of the chain's. Negligible; mention it if internal rotors come up again.
8. **Experiment support.** `../experiments/README.md` lists what the flight experiments need that the sim can't do yet
   (an anchored end, an IMU-style output, partial-g stall tests).

## Physics choices
- **Meshes and collision:** meshes are visual only (`mass=0`, no contacts). Each base collides as one sphere;
  neighbouring bases are excluded from colliding.
- **Mass:** the sphere carries `shell_frac × base_mass` with `shellinertia`. The rest of the base's mass sits on a core
  box, and the arm has its own box. Sizes are in `make_chain.py`.
- **Servo model:**
  - `kp = stall / 10°` and `forcerange = ±stall`.
  - Damping = stall torque / no-load speed, so the no-load speed emerges when the servo saturates.
  - `timeconst` = 5 ms lag on targets.
  - **Anti-wobble (2026-10-20):**
    - Three additions: `frictionloss` = `GEAR_FRICTION` (7%) × stall torque, a per-preset `armature` (0.002 / 0.0002
      kg·m²), and actuator `kv` (1.0 / 0.25).
    - `kv` sits inside the force clamp, so it only brakes near the target.
    - Effect on a 60° step at m6: before, 9.5° overshoot, 15 swings, 3.5 s to settle, 8° flex. After, 1°, 2 swings,
      0.5 s, 2.7°.
    - **`armature` is what makes `kv` possible** at a 0.5 ms timestep; without it the light twist mode goes unstable.
    - All three are internal, so momentum is conserved and the zero-g split is unchanged.
- **7 cm presets:** lengths × 0.5. Printed mass × 0.5³ plus real servo masses (XL330 18 g, STS3032 about 20 g).
- **Integrator: RK4.**
  - implicitfast drifted the centre of mass 135 mm in 12 s of random motion; RK4 drifts 0.005 mm.
  - Stability needs timestep < ~2.8·(I_twist + armature)/(4·(damping + kv)): 0.5 ms for original14 and 0.1 ms for the
    7 cm presets.
  - Without the 5 ms lag, 10 ms target steps make conservation about 100× worse.
- **Floor contact:** `condim 6` with rolling friction = `ROLLING_RESISTANCE` × R. Without it, the bases' ~7 mm
  centre-of-mass offset makes a dropped chain rock forever, because energy is conserved.
- **World layout:**
  - The chain floats along world +Y at height n·pitch/2 + 2R, so a zero-g fold can't reach the floor at z = 0.
  - `CHAIN_ROLL = 0` makes the pitch axes vertical, so pitch folds lie flat on the floor.
  - Gravity defaults to 0.
- **Ranges:** pitch ±124°, yaw ±150°. `--slip-ring` frees yaw only, with targets spanning ±360°.

## Interface details
- `physics` freezes the whole sim through `Session.running`; `pause` only holds the fold sequence.
- **`spin`** adds a rigid rotation about the centre of mass to the free joint.
  - Root linear velocity += Δω × (p_root − c), and root angular velocity += Δω.
  - That sets (I⁻¹L)_z to the rate while leaving the shape and joint velocities alone.
  - It's an outside push, so it changes L.
- **Readouts:**
  - `spin` = (I_total⁻¹ L)_z, the whole chain's rate if its shape froze.
  - `turned` = the heading change of m6's chain axis about the vertical since the last reset. The module-1-to-12 line
    was rejected because a tip swing alone moves it about 5°.
- **Timing:** `Session` keeps a sequence clock separate from sim time (for speed, pause and restart). Targets update
  and the log writes every 10 ms (`mj_step(nstep=…)`).
- **Screenshots** use an offscreen `mujoco.Renderer` with the viewer's camera.
- **Viewer keys:** PageUp/PageDown select a module, Up/Down nudge pitch, Right/Left nudge yaw, Space pauses physics.
- **Tested in the real GUI:** Play (2026-10-17), layout screenshots, and Spin → Play → Stop spin (2026-10-21).

## Meshes
- **Grouping:** by layer (`Link::*` → arm, everything else → base). The module comes from the height of each object's
  centre (rounded for the base; floored with a +20 mm bias for the arm).
- **Decimation is vertex clustering, not quadric.** Quadric decimation failed on the open, 165-piece servo meshes and
  moved parts by up to 9.6 mm. Shells keep full resolution; the grid is chosen by bisection (about 1.6–2.0 mm).

## Gotchas for agents
- **macOS viewer:** it needs `mjpython`. Tk can't run inside mjpython because both need the main thread; that's why
  the panel is a separate process.
- **Don't touch other people's windows.** Check `ps` for `panel.py|play.py` first, and launch your own instance to test.
- **The passive viewer ignores `<visual><global azimuth/elevation>`.** `run_viewer` calls `mjv_defaultFreeCamera`
  instead, and `statistic extent` = the chain length, so the whole chain fits.
- **Screenshots (macOS):**
  - The mjpython viewer can be granted to screen tools (`org.mujoco.mjpython`).
  - The Tk panel can't: a plain-python process has no app bundle.
  - Workaround for the panel:
    1. Copy the uv `python3.11` into a throwaway `X.app/Contents/MacOS/` with an Info.plist bundle id.
    2. `lsregister -f` it, and run it with `PYTHONHOME=<uv base prefix>`.
    3. Delete it and `lsregister -u` afterwards.
- **Scripted GUI tests can press real buttons:** find the `ttk.Button` by its text, call `.invoke()`, and run with
  `python -u`.
- **Clean up after experiments:**
  - `MUJOCO_LOG.TXT` appears when a run goes unstable; delete it afterwards.
  - Remove `__pycache__/` after test runs.

## Verification
`verify.py` has 7 sections:
1. Compile, and meshes match the current CAD.
2. Joint axes and arm mesh.
3. Zero-g momentum.
4. Contacts.
5. Gravity drop.
6. Spin.
7. Renders.

Its docstring says what each checks, and the key numbers are under Contracts.
