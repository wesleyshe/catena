# Fold sequences

A fold sequence is a scripted move: a CSV of joint angles, one keyframe per row. The simulation plays these files, and
the robot software should read the same format, so one file runs in both.

| File | What it does |
|---|---|
| `fold_sequence.csv` | Straight → L → straight → zig-zag → straight. `verify.py` uses it, so don't change it without re-running the checks. |
| `reorient_loop.csv` | Turns the whole chain about 30° with no push: 4 shape loops that each end straight, keeping angular momentum at 0 (the "falling cat"). |

Add your own here. The control window lists every `.csv` in this folder.

## Format
- **Angles:** each row is one keyframe of 24 angles in **degrees**, in the order
  `m1_yaw, m1_pitch, m2_yaw, m2_pitch, …, m12_yaw, m12_pitch`.
- **Header and comments:** a header row naming the columns is optional. Lines starting with `#` are skipped, so use
  them to say what the sequence does.
- **Columns with no servo:** `m1_yaw` and `m12_pitch`. Keep them at 0; the sim warns if they aren't.
- **Ranges:** pitch ±124° and yaw ±150°. The sim clamps targets to these.
- **Playback:** the sim holds each row for `hold` seconds, then eases to the next over `move` seconds with a cosine
  profile. The defaults are `move` 3 s and `hold` 1 s, set by `--move` and `--hold` or in the control window. Targets
  update every 10 ms, like commands on a servo bus.

## Try one
```bash
cd simulation
.venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless --log logs/fold.csv
```
Or double-click `simulation/Chain Sim.command`, pick the file under **Fold sequence**, and press **Play**.
