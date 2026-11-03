# Fold sequences

A fold sequence is a scripted move: a CSV of joint angles, one keyframe per row. The simulation plays these files, and
the robot software should read the same format, so one file runs in both.

**A sequence is only the moves.** Everything else about a run is in a *setup file* (`../simulation/setups/`): the
timing, gravity, a rod or pole, the end effectors and where they start, the spin at release. So some sequences play on
the bare chain anywhere, and others only make sense with the setup they were written for. For example, the pole in the
pole climb is the `[rod]` table of setup `5a_pole_climb`, not part of `pole_climb.csv`; play the CSV on its own and the
chain just coils up in mid-air. In the control window, picking a sequence shows which setups use it.

Every setup now flies them as they'll fly: in the box on the plane, held by strings (or an anchor) through the 1.8 g
pull-up, let go about 1 s into zero g, the sequence starting at T+1.5 s, the pull-out dropping the chain on the net. So
a sequence has about 20 s (`parabola_s` is 22 s), and sequences are angles, so they play on any module size: the setups
use the 8.5 cm flight build.

### Bare chain: these play with any setup
| File | What it does | Setups that use it |
|---|---|---|
| `fold_sequence.csv` | Straight → L → straight → zig-zag → straight. `verify.py` uses it, so don't change it without re-running the checks. | `6a_skater` |
| `reorient_loop.csv` | Turns the whole chain about 30° with no push: 4 shape loops that each end straight, keeping angular momentum at 0 (the "falling cat"). At default timing this takes 65 s. At `--speed 4` it takes 16 s, fitting one parabola: it turns about 31°, with servos peaking at 1.28 of 1.5 N·m. | `1a_turn`, `1b_from_net`, `1c_spin_*`, `1e_turn_across` (at flight timing, move 1 s, hold 0.25 s) |
| `roll_loop.csv` | Rolls the whole chain about its own long axis with no push: 2 loops of an m4 pitch fold plus an m8 twist (a 3D "falling cat"), about 64° in total. Default timing takes 33 s; `--speed 2` fits one parabola (16.5 s, 63.4°). Reverse the steps to roll the other way. | `1d_roll` |

### Written for a setup: they need its end effectors or world
| File | Setup | Needs | What it does |
|---|---|---|---|
| `arm_reach.csv` | `2a_arm` | an anchor on ee1, on a wall (the box's end wall) | Curls, turns a quarter turn on ee1's twist servo (`m1_yaw`), reaches, folds into an L, straightens. 18.5 s at move 2.5 s, hold 1 s. |
| `stand_out.csv` | `2b_stall_*` | an anchor on ee1, on a wall, pitch axes level (the 2b setups fly it in 0 g, Moon and Mars parabolas and at 1 g) | Hanging from m1_pitch (−90°), told to stand straight out; how far it gets at each gravity is the stall limit (also `simulation/stall_sweep.py`). 20 s at move 4 s, hold 4 s. |
| `arm_let_go.csv` | `2c_let_go` | an anchor on ee1, on a wall | A fast swing, then `ee1_grip` 0 while the servos are still catching up, so it drifts off with that momentum. 19.5 s at move 0.5 s, hold 3.5 s. |
| `thrust_straight.csv`, `thrust_U.csv`, `thrust_Z.csv`, `thrust_L.csv` | `3a_*` | thrusters on both ends | Fold into the shape, fire both end nozzles (n0) at half throttle for 2.5 s, then coast. Straight: nothing; U: glides; Z: spins in place; L: both. 18.5 s at move 1.5 s, hold 1 s. |
| `inchworm_walk.csv` | `4a_walk` | suction cups on both ends, ee1 on a flat surface (in 4a, a deck laid across the box over the net) | An inchworm walk: a foot lets go only when the other holds; each landing foot is aimed 15 mm into the floor so its cup presses on. About 0.56 m per step; 19 s at move 0.9 s, hold 0.35 s. |
| `walk_let_go.csv` | `4b_walk_let_go` | the same as `4a_walk` | The same walk, but the back foot lets go before the front one holds, so it floats off. 19 s. |
| `pole_climb.csv` | `5a_pole_climb` | a pole sized to the modules (Ø100 mm for 14 cm, Ø61 mm for 8.5 cm), the chain starting wrapped round it (`[rod] wrap_m`) | The bare chain climbs the pole like a snake: three zones of four modules, the ends grip by squeezing while the middle stretches and shortens. Row 0 is the coil touching the pole all round. Written by `simulation/pole_climb_design.py`. About 13 cm per step, ~39 cm in 19.3 s at move 0.7 s, hold 0.3 s. |
| `rod_crawl.csv` | `5b_rod_crawl` | anchors on both ends, ee1 on a rod (a Ø25 mm rail across the box in 5b) | Clamps crawl along the rod with the same arch gait as the walk. About 0.56 m per step; 19 s at move 0.9 s, hold 0.35 s. |

**Adding one:** put the CSV here and say what it does in its first `#` line (the control window shows that line). If
it needs end effectors or a world, save a setup for it too (see "Make your own run" in `../simulation/README.md`) and
add a row to the second table. The control window lists every `.csv` in this folder.

## Format
- **Angles:** each row is one keyframe of 24 angles in **degrees**, in the order
  `m1_yaw, m1_pitch, m2_yaw, m2_pitch, …, m12_yaw, m12_pitch`.
- **Header and comments:** a header row naming the columns is optional. Lines starting with `#` are skipped, so use
  them to say what the sequence does.
- **Columns with no servo:** `m12_pitch`, and `m1_yaw` unless the chain has an end effector past m1. Keep them at 0;
  the sim warns if they aren't.
- **`m1_yaw` with an end effector past m1** drives that end effector's twist servo (±150°). Positive turns the m12 side
  the same way as every other yaw. Sequences written for the bare chain leave it at 0, so they still work.
- **End-effector columns** (optional, after the 24 joint columns; the file must have a header row):
  - `ee1_grip`, `ee12_grip`: 1 = hold (an anchor or suction cup grabs whatever fixed thing it touches), 0 = let go.
  - `ee1_n0` … `ee1_n4`, `ee12_n0` … `ee12_n4`: thruster nozzles, throttle 0–1. n0 blows out of the end along the
    chain; n1–n4 blow out sideways along the end module's +x, +y, −x, −y. Each pushes the opposite way.
  - A row's values apply from the start of that row's hold (when the joints have arrived) until the next row's hold.
    They don't ease. A missing column leaves that end as it started; a column for an end that can't do it is ignored
    with a warning.
  - The robot software should read the same columns as end-effector commands.
- **Ranges:** pitch ±124° and yaw ±150°. The sim clamps targets to these.
- **Playback:** the sim holds each row for `hold` seconds, then eases to the next over `move` seconds with a cosine
  profile. The defaults are `move` 3 s and `hold` 1 s, set by `--move` and `--hold` or in the control window. Targets
  update every 10 ms, like commands on a servo bus.
- **Speed:** `--speed 4` (or **Speed x** in the control window) plays the whole sequence 4× faster.
- **Flight sequences** must fit one parabola, about 22 s (`parabola_s` in `../hardware/spec.toml`), less the time
  before the chain is let go (`release_s` in a setup's `[flight]`). The sim flags
  longer ones. Faster than about 3 s per loop the servos can't keep up, and the result changes; see
  `../experiments/findings.md`.

## Try one
```bash
cd simulation
.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless --log logs/fold.csv
```
Or double-click `simulation/Chain Sim.command`, pick the file under **Fold sequence**, and press **Play**.
