# AGENTS.md: software

Read the root `AGENTS.md` first. This folder is for the code that runs the real chain. History is in `LOG.md`.

## Status (2026-10-22)
- **Empty.** No code for the real chain is in this repo yet.
- **Still to do, per the proposal:**
  - the logging and telemetry stack
  - reading an IMU
  - playing fold sequences on the hardware
- The folding algorithm already exists, outside this repo.
- **Owner:** (add your name).
- **When code lands, replace this section** with what exists, how to set it up, and how to test it.

## The hardware, as software sees it
- **Servos:** 22 Dynamixel AX-12A servos, which speak Dynamixel Protocol 1.0, driven through a U2D2 in an end module.
- **Joint names and CSV column order:** see the root `AGENTS.md`.
- **Numbers:** read them from `../hardware/spec.toml` (in Python, `tomllib.load`). Don't copy them into code; then a
  hardware change reaches your code on the next pull.
- **Ranges:** pitch ±124°, yaw ±150° (`[joints]` in the spec).
  - Past ±124° the hub hits the end of the shell slot, so commanding beyond it would push the servo against the shell.
  - The sim clamps targets to these ranges. The real servos won't, unless you set their angle limits.
- **Not recorded anywhere yet:**
  - the servo IDs
  - which way each servo turns
  - how the straight pose (all joints 0°) maps to servo position units
  Measure them on the chain and add them to `../hardware/spec.toml`, so the sim can use them too.

## Interfaces with the rest of the repo
- **Fold sequences (`../sequences/`):** a format both the sim and the robot can read; see `../sequences/README.md`. If
  your code writes or plays sequences, use this format so one file runs in both.
- **Logs:**
  - The sim logs every 10 ms: time, centre of mass, angular momentum, max servo torque and contacts. The columns are in
    `../simulation/README.md`.
  - Suggestion, not decided: if the chain's own log records time and joint angles at a similar rate, flight data can be
    compared directly with the sim's prediction.

## Try a sequence in the sim before running it on the chain
Set up the sim once; the steps are in `../simulation/README.md`. Then:
```bash
cd simulation
.venv/bin/python play.py --seq ../sequences/my_fold.csv --headless --log logs/my_fold.csv
```
In the log:
- `contacts` should stay 0. Anything else means modules hit each other.
- `max_torque` at 1.5 N·m means a servo is at its limit, either stalled or moving at full speed.

To watch it instead, double-click `simulation/Chain Sim.command` and pick the file under **Fold sequence**.

## Checks
- None yet. Add the command that tests your code here once it exists.
