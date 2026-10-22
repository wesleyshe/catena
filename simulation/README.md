# Zero-gravity chain sim

A free-floating MuJoCo model of the MIT Universal Self-Folding Chain Robot (Mutis et al., ACM SCF 2025).
By default there is no gravity and no tether. You can drive it with sliders and the keyboard, or play folding
sequences, and log how its centre of mass and angular momentum behave. The chain floats horizontally about 1 m above
a floor; turn gravity on and it drops onto the floor.

![one module](renders/module.png)

## Setup (once)

Run all of the commands below from this `simulation/` folder. Setup needs [uv](https://docs.astral.sh/uv/).

```bash
cd simulation
```

```bash
uv venv --python 3.11 .venv
```

```bash
uv pip install --python .venv/bin/python -r requirements.txt
```

On a Mac the MuJoCo viewer needs `mjpython`, which ships in `.venv/bin/`. Everything that doesn't open a window runs
with plain `.venv/bin/python`.

## Control window (no shell needed)

Double-click **`Chain Sim.command`** in Finder. A Terminal window opens and must stay open, because it runs the app.
The first time, macOS may ask whether Terminal can access the Desktop folder; allow it. Then the **Chain Sim**
control window and the MuJoCo viewer open side by side.

| Section | What it does |
|---|---|
| Model | Pick the preset, number of modules and slip ring, then **Rebuild + open viewer**. This rewrites `chain.xml` and restarts the viewer. |
| Fold sequence | Pick a CSV from `../sequences/` (or **Browse...**), set move/hold seconds and speed, then **Play / Pause / Resume / Restart / Stop**. |
| World | **Gravity** on/off and its strength, **Straighten** (all joints to 0), **Reset pose** (floating start, at rest), and **Pause physics / Resume physics**, which freezes the whole simulation (time, motion, sequence, log) so you can look around. |
| Spin | Set a rate in deg/s and press **Spin** to start the whole chain spinning about the vertical through its centre of mass, keeping its shape. **Stop spin** brings it to rest. This is a push from outside: it gives the chain angular momentum, which then stays constant. Fold while spinning and it speeds up, like a skater pulling their arms in. |
| Record | **Log to logs/** writes `logs/run_<date>_<time>.csv` while ticked. **Screenshot** saves the viewer's view to `screenshots/`. **Open folder** opens this folder in Finder. |
| Readouts | Time, sequence progress, gravity, **spin** (the whole chain's spin rate about the vertical: angular momentum ÷ inertia), **turned** (how far the middle module has turned about the vertical since the last reset; twists about the chain's own axis don't count), centre-of-mass drift, total angular momentum, max servo torque, contacts and log file. These are also drawn in the top-left of the viewer. |

The viewer's own sliders and keys still work alongside it. Closing the control window also closes the viewer.

## Commands

| What | Command |
|---|---|
| Summarise a .3dm file (units, layers, objects, boxes) | `.venv/bin/python inspect_3dm.py "../hardware/cad/Single module existing.3dm"` |
| Re-export the STLs from `../hardware/cad/complete chain.3dm` | `.venv/bin/python export_meshes.py` |
| Generate `chain.xml` (12 × original14) | `.venv/bin/python make_chain.py` |
| Viewer: sliders + keyboard | `.venv/bin/mjpython play.py` |
| Viewer: play the fold sequence | `.venv/bin/mjpython play.py --seq ../sequences/fold_sequence.csv` |
| No viewer: run the sequence and log | `.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless` |
| Viewer with Earth gravity (drops to the floor) | `.venv/bin/mjpython play.py --gravity 9.81` |
| Run all checks and renders | `.venv/bin/python verify.py` |
| Control window (same as double-clicking `Chain Sim.command`) | `.venv/bin/python panel.py` |

`play.py` options: `--model FILE.xml` (without it, the model is built fresh from `../hardware/spec.toml`), `--seq FILE`, `--move 3` (seconds between rows), `--hold 1` (seconds per row),
`--log logs/run_log.csv`, `--headless`, `--gravity 9.81` (m/s² towards the floor, overriding the model).

### Keyboard (viewer)

| Key | Action |
|---|---|
| PageUp / PageDown | select next / previous module (the selected one turns orange) |
| Up / Down | pitch of the selected module ±5° |
| Right / Left | yaw of the selected module ±5° |
| Space | pause / resume the physics (the viewer's own Pause button is greyed out) |

You can also drag the sliders in the viewer's **Control** panel on the right. While a sequence is playing, the sequence
overrides both the keys and the sliders. If a key clashes with a viewer shortcut, change `KEYS` at the top of `play.py`.

## Gravity and the floor

- **Default: zero gravity.** The chain floats high enough that no fold can reach the floor: more than half its
  length plus a shell diameter. The floor is just a visual reference.
- **To drop it,** use `play.py --gravity 9.81`, or `make_chain.py --gravity 9.81` to build gravity into the XML. You can
  also edit **Physics → gravity** in the viewer's left panel. That field is an x y z vector: Earth is `0 0 -9.81`.
  Something like `1 1 1` pulls diagonally upwards, which is why it drifted sideways.
- **Lying on the floor,** the chain runs along world +Y with its pitch axes vertical, so pitch folds lie flat on the
  floor. Change `CHAIN_ROLL` in `make_chain.py` (90°) to make pitch lift modules off the floor instead.
- **Rolling resistance.** Each module's centre of mass is about 7 mm off its shell centre, so a dropped module behaves
  like a lopsided ball. The floor therefore has rolling resistance (`ROLLING_RESISTANCE = 0.03`) so the chain settles
  within about 10 s. This value is tuned for settling, not measured. Below about 0.01 it keeps rocking.

## Switching presets

```bash
.venv/bin/python make_chain.py --preset dyna7 -o chain_dyna7.xml
```

```bash
.venv/bin/mjpython play.py --model chain_dyna7.xml --seq ../sequences/fold_sequence.csv
```

| Preset | Servo | Stall torque | Pitch | Mesh scale | Timestep |
|---|---|---|---|---|---|
| `original14` | Dynamixel AX-12A | 1.5 N·m | 14 cm | 1.0 | 0.5 ms |
| `dyna7` | Dynamixel XL330-M288 | 0.52 N·m | 7 cm | 0.5 | 0.1 ms |
| `feetech7` | Feetech STS3032 | 0.44 N·m (peak) | 7 cm | 0.5 | 0.1 ms |

Other flags: `-n 8` sets the number of modules (minimum 2), and `--slip-ring` removes the yaw limits. Yaw targets then span ±360°, which
keeps the sliders usable. Pitch stays at ±124° because the shell, not the cable, limits it. The real chain's numbers (masses, sizes, ranges,
servo) are in `../hardware/spec.toml`. The what-if presets and the sim's own tuning are in `PRESETS` and the constants at
the top of `make_chain.py`. Run `verify.py` again after changing either.

## Files

| File | What it does |
|---|---|
| `../hardware/cad/*.3dm` | The CAD inputs (mm): `Single module existing.3dm`, `complete chain.3dm`. These are never modified. |
| `inspect_3dm.py` | Prints units, layers, objects, colours, types and tight mesh bounding boxes. |
| `export_meshes.py` | Writes `meshes/*.stl`, one per rigid body, in the module frame, in metres, under 20k triangles. |
| `meshes/` | `base_first.stl` (module 1), `base_mid.stl` (2–11), `base_last.stl` (12), `arm.stl` (1–11). |
| `../hardware/spec.toml` | The real chain's numbers, shared with the robot software. `make_chain.py` reads them. |
| `make_chain.py` | Parametric generator for `chain.xml`. |
| `chain.xml` | Generated model, written by the control window and `make_chain.py`. Not in git. |
| `meshes/SOURCE.txt` | Which CAD file the meshes were exported from; `verify.py` fails if the CAD has changed since. |
| `Chain Sim.command` | Double-click launcher for the control window. |
| `panel.py` | Control window: buttons that build the model and drive the viewer. |
| `play.py` | Viewer, keyboard, sequence playback, logging, live readouts; takes commands from `panel.py`. |
| `../sequences/` | Fold-sequence CSVs, shared with the robot software; the control window lists these. `fold_sequence.csv` goes straight → L → straight → zig-zag → straight. `reorient_loop.csv` turns the whole chain ~30° with no push (4 shape loops that each end straight). Add your own here. |
| `logs/` | Run logs: `run_log.csv` from the shell, `run_<date>_<time>.csv` from the control window. |
| `screenshots/` | Written by the control window's **Screenshot** button. |
| `verify.py` | Checks plus PNG renders in `renders/`. |
| `renders/` | `module.png`, `module_pitch60.png`, `chain12.png`, `fold_L.png`, `fold_zigzag.png`, `gravity_drop.png`. |
| `AGENTS.md`, `LOG.md` | Decisions behind the model (for coding agents), and the history of changes. Measurements are in `../hardware/AGENTS.md`. |

## The model in brief

- **Two rigid bodies per module.** The **base** is the shell, bracket and both servos; it's one body because the
  connector pins lock the core to both shell halves. The **arm** is the U-yoke and hub.
- **Pitch** is about +X through the module centre: the pitch servo's horn turns the yoke. **Yaw** is about +Z: module
  *i*'s hub bolts onto module *i+1*'s downward-facing yaw horn. These axes are in each module's own frame. With all
  joints at zero the chain is straight, 140 mm centre to centre, and in the world it lies along +Y.
- **In zero g a joint turns both sides, split by how hard each side is to turn.** That's moment of inertia, mass ×
  distance², not mass alone. The relative angle is always what you commanded, and the centre of mass doesn't move.
  Measured with one joint commanded to 60°:

  | Joint | Side before the joint | Side after |
  |---|---|---|
  | m6_pitch, all modules equal | −33.7° | +26.3° (modules 7–12 extend one pitch further) |
  | m6_pitch, modules 1–6 twice as heavy | −30.6° | +29.4° |
  | m6_pitch, modules 7–12 twice as heavy | −36.4° | +23.5° |
  | m3_pitch (3 modules vs 9) | −54.1° | +5.9° |
  | m1_pitch (1 module vs 11) | −60.0° | 0.0° |

  So end joints move the end module, not the chain. m1_pitch spins module 1's whole base (shell, servo, U2D2), and
  that's real behaviour, not a disconnected part.
- **22 actuated joints, not 24.** Module 1 has no yaw servo; it carries the U2D2 and power hub instead. Module 12 has
  no pitch servo or arm. Both of these come from `complete chain.3dm`.
- **Visuals vs physics.** Meshes are visual only. Each module collides as one sphere (Ø139 mm). Mass sits on simple
  primitives: a hollow sphere for the shells, plus a box for the core and a box for the arm.
- **Servos** are position actuators with a force limit equal to stall torque. Stall torque is reached at 10° of
  position error. Damping is stall torque divided by no-load speed, so a saturated servo moves at its rated speed.
  A 5 ms lag smooths each target.
- **No springy wobble.** Each joint also has gearbox friction (7% of stall torque), the motor's inertia through the
  gearbox (`armature`), and braking as it nears its target (`kv`).
  - A 60° jump at m6 now overshoots about 1° and settles in 0.5 s. Before, it overshot 9.5° and swung 15 times.
  - These values are estimates. Raise `kv` in `PRESETS` for stiffer, slower stops, or lower it for snappier ones.
- **Integrator.** It uses RK4, because the default integrator let the centre of mass wander tens of millimetres during
  random motion. RK4 is only stable below a timestep limit, so lighter modules need a smaller timestep (see AGENTS.md).

### Fold sequence CSVs (`../sequences/`)

The full format, shared with the robot software, is in `../sequences/README.md`. In brief:

Each row is one keyframe of 24 angles in degrees, in the order `m1_yaw, m1_pitch, …, m12_yaw, m12_pitch`. A header
row naming the columns is optional, and lines starting with `#` are skipped. `m1_yaw` and `m12_pitch` have no servo,
so they're ignored, with a warning if they're not zero. Rows are eased between with a cosine profile over `--move`
seconds, and each row is held for `--hold` seconds. Targets update every 10 ms, like commands on a servo bus.

### Log files (`logs/`)

Columns: `time`, `com_x/y/z` (chain centre of mass, m), `L_x/y/z` and `L_norm` (total angular momentum about the
centre of mass, kg·m²/s), `max_torque` (largest |actuator torque|, N·m) and `contacts` (number of contacts).
One row is written every 10 ms.

## Verified (`verify.py`)

- All three presets compile, with and without `--slip-ring`.
- Every joint's axis and anchor is correct.
- A 30° test turn moves everything downstream rigidly about the right axis.
- The yoke discs are centred on the pitch axis (within 0.002 mm) and the hub on the yaw axis (within 0.014 mm).
- 12 s of smooth random motion: the centre of mass moves ≤ 0.005 mm, and |L| stays below 10⁻⁷ of the summed body
  momenta.
- No contacts with the chain straight (neighbouring spheres are 1 mm apart), and none during the fold sequence.
- With gravity at 9.81 m/s², every preset drops onto the floor and settles within 15 s, with all 12 shells resting
  on it.
