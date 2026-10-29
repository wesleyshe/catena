# Fold sequences

A fold sequence is a scripted move: a CSV of joint angles, one keyframe per row. The simulation plays these files, and
the robot software should read the same format, so one file runs in both.

| File | What it does |
|---|---|
| `fold_sequence.csv` | Straight → L → straight → zig-zag → straight. `verify.py` uses it, so don't change it without re-running the checks. |
| `reorient_loop.csv` | Turns the whole chain about 30° with no push: 4 shape loops that each end straight, keeping angular momentum at 0 (the "falling cat"). At default timing this takes 65 s. At `--speed 4` it takes 16 s, fitting one parabola: it turns about 31°, with servos peaking at 1.28 of 1.5 N·m. |
| `arm_reach.csv` | For a chain anchored by ee1 (setup `2a_arm`): curls, turns a quarter turn on ee1's twist servo, reaches, folds into an L, straightens. 18.5 s at move 2.5 s, hold 1 s. |
| `arm_let_go.csv` | Anchored by ee1 (setup `2c_let_go`): a fast swing, then `ee1_grip` 0 while the servos are still catching up, so it drifts off with that momentum. 19.5 s at move 0.5 s, hold 3.5 s. |
| `stand_out.csv` | Hanging from m1_pitch (−90°), told to stand straight out (setups `2b_stall_*`, and `simulation/stall_sweep.py`). 20 s at move 4 s, hold 4 s. |
| `inchworm_walk.csv` | An inchworm walk on two suction cups (setup `4a_walk`): a foot lets go only when the other holds; each landing foot is aimed 15 mm into the floor so its cup presses on. About 0.56 m per step. 19 s at move 0.9 s, hold 0.35 s. |
| `walk_let_go.csv` | The same walk, but the back foot lets go before the front one holds (setup `4b_both_off`). 19 s. |
| `roll_loop.csv` | Rolls the whole chain about its own long axis with no push: 2 loops of an m4 pitch fold plus an m8 twist (a 3D "falling cat"), about 64° in total. Default timing takes 33 s; `--speed 2` fits one parabola (16.5 s, 63.4°). Reverse the steps to roll the other way. |

Add your own here. The control window lists every `.csv` in this folder.

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
- **Flight sequences** must fit one parabola, about 20 s (`parabola_s` in `../hardware/spec.toml`). The sim flags
  longer ones. Faster than about 3 s per loop the servos can't keep up, and the result changes; see
  `../experiments/findings.md`.

## Try one
```bash
cd simulation
.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless --log logs/fold.csv
```
Or double-click `simulation/Chain Sim.command`, pick the file under **Fold sequence**, and press **Play**.
