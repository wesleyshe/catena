# Fold sequences

A fold sequence is a scripted move: a CSV of joint angles, one keyframe per row. The simulation plays these files, and
the robot software should read the same format, so one file runs in both.

| File | What it does |
|---|---|
| `fold_sequence.csv` | Straight → L → straight → zig-zag → straight. `verify.py` uses it, so don't change it without re-running the checks. |
| `reorient_loop.csv` | Turns the whole chain about 30° with no push: 4 shape loops that each end straight, keeping angular momentum at 0 (the "falling cat"). At default timing this takes 65 s. At `--speed 4` it takes 16 s, fitting one parabola: it turns about 31°, with servos peaking at 1.28 of 1.5 N·m. |
| `roll_loop.csv` | Rolls the whole chain about its own long axis with no push: 2 loops of an m4 pitch fold plus an m8 twist (a 3D "falling cat"), about 64° in total. Default timing takes 33 s; `--speed 2` fits one parabola (16.5 s, 63.4°). Reverse the steps to roll the other way. |

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
