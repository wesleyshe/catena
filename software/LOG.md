# Log: software

Newest first. The format is in the root `AGENTS.md`.

## 2026-10-22 · Wesley (Claude Code)
- **The sim log now has IMU columns:** quaternion, and gyro in rad/s. Matching them in the chain's own log makes
  comparison easy.
- **The flight servos will be STS3032:** a single lead each, Feetech protocol. Recording the IMU's rate at release is
  now on the to-do list, because the sim's predictions depend on it.

## 2026-10-22 · Wesley (Claude Code)
- `../hardware/spec.toml` now holds the chain's numbers (ranges, servo, masses). Read it from code rather than copying
  values, and add servo IDs and offsets there once measured.

## 2026-10-22 · Wesley (Claude Code)
- Created this folder for the robot software. It's empty so far; `AGENTS.md` lists the interfaces the code should
  match.
