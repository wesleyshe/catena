# Log: repo-wide and shared folders

Newest first. This file covers root files, `sequences/`, `experiments/` and `docs/`. Each area has its own `LOG.md`.
The format is in the root `AGENTS.md`.

## 2026-10-29 · Wesley (Claude Code)
- **New sequence** `pole_climb.csv`: the bare chain wrapped around a pole climbs it, gripping by squeezing (setup
  `5_pole_climb`, demo button 5). `experiments/findings.md` gains the squeeze-grip numbers.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-29 · Wesley (Claude Code)
- **`experiments/findings.md`: "The class simulations".** The six clips' measured results; the stall length per
  gravity (3 / 4 / 6 / 12+ modules at Earth / Mars / Moon / zero g, experiment A's number); a suction cup is sized by
  tipping, not pull (~2.4 N·m: 80 mm, 60 N walks, 40 mm, 20 N peels); the side nozzles roll the chain a little because
  each module's centre of mass sits ~4.8 mm off its axis. `experiments/README.md` updated to match.

## 2026-10-29 · Wesley (Claude Code)
- **New sequence** `rod_crawl.csv`: an inchworm crawl along a rod with a clamp on each end (setup `5a_rod_crawl`).
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-29 · Wesley (Claude Code)
- **New sequences** for the thruster clips: `thrust_straight.csv`, `thrust_U.csv`, `thrust_Z.csv`, `thrust_L.csv`
  (fold, a brief burst of both end nozzles via `ee1_n0` / `ee12_n0`, coast).
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-29 · Wesley (Claude Code)
- **New sequences** for the suction-cup clips: `inchworm_walk.csv` (an inchworm walk, one foot always holding) and
  `walk_let_go.csv` (the same, letting go too early). They use the `ee1_grip` / `ee12_grip` columns.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-29 · Wesley (Claude Code)
- **Shared format: sequences can now carry end-effector columns** after the 24 joint columns: `ee{k}_grip` (1 hold,
  0 let go) and `ee{k}_n0` … `n4` (thruster throttle 0–1). They apply from the start of a row's hold. Documented in
  `sequences/README.md` and the root `AGENTS.md`. **Software needs to:** read them as end-effector commands once the
  real end effectors exist; until then, ignore columns starting with `ee`.
- **New sequences** for the anchored-arm clips: `arm_reach.csv`, `arm_let_go.csv`, `stand_out.csv`.
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-29 · Wesley (Claude Code)
- **Shared fact changed: `m1_yaw` can now be a real servo.** In the sim, an optional end-effector module past m1 sits
  on a twist servo, and that servo is `m1_yaw` (same sign as every yaw, ±150°): 23 joints with it fitted, 22 without.
  The sequence format is unchanged; existing sequences keep `m1_yaw` at 0 and still work.
  - Updated: root `AGENTS.md` (shared facts), `sequences/README.md`.
  - **Software needs to:** treat the `m1_yaw` column as a servo command when the chain has the ee1 end effector, and
    ignore it otherwise. End-effector command columns (grip, nozzles) come in a later phase.
- **Added the end-effector design and its phase 1 plan** in `docs/design/` (approved by Wesley 2026-10-29).
- **Checked:** `simulation/verify.py` ALL CHECKS PASSED; details in `simulation/LOG.md`.

## 2026-10-22 · Wesley (Claude Code)
- **Answered a team question in the sim:** one fold vs. loops of folds, in 3D.
  - One fold leaves no lasting turn (0.02°).
  - Pitch-only loops turn the chain about the vertical only (7.4° per loop).
  - A pitch fold plus a twist turns it about its long axis (31.8° per loop).
  - The CG stayed fixed (0.0000 mm) throughout.
  - Added to `experiments/findings.md`.
- **Added `sequences/roll_loop.csv`:** 2 pitch-plus-twist loops, about 64°. Fits one parabola at `--speed 2`.
- **`.gitignore` now ignores `.gstack/`.** The gstack browser keeps logs and a local access token there, which must
  never be pushed to this public repo.
- **Checked:** `roll_loop.csv` at three speeds: 0 contacts, peak servo torque at most 0.99 of 1.5 N·m, CG fixed.

## 2026-10-22 · Wesley (Claude Code)
- **Published as a private GitHub repo:** https://github.com/wesleyshe/catena.
- **Squash-merge only.**
- **New `verify` check** (`.github/workflows/verify.yml`): runs `simulation/verify.py` on Ubuntu, with software OpenGL
  for the renders, on every pull request and on pushes to main.

## 2026-10-22 · Wesley (Claude Code)
- **Outside physics review of the sim,** plus sim runs of flight conditions. Results are in `experiments/findings.md`
  under "Is the sim good enough for flight?":
  - The core is sound.
  - Spin at release dominates: 1°/s cancels a 30° turn.
  - Battery placement shifts the turn by 9%.
  - Air drag shifts it by 5%.
  - The servo settings shift it by 4% or less.
- **Corrected findings that claimed too much:** they're predictions, the turn depends on speed once the servos lag, and
  the −0.15° per cycle is real.
- **`experiments/README.md`:** experiment A now lists what the sim covers and what the flight plan must handle (IMU rate
  at release, stowing before the 1.8 g pull-out, about 20 s per parabola).
- **`sequences/README.md`:** timing for flight; `reorient_loop.csv` fits a parabola at `--speed 4`.

## 2026-10-22 · Wesley (Claude Code)
- **Rehearsed a day of team work** with hardware, 2 × software and simulation members opening pull requests into
  main. GitHub's merge button was simulated as ignoring merge drivers.
- **Before:** 3 of 4 pull requests hit conflicts, all from:
  - `LOG.md` entries
  - neighbouring status lines in the root `AGENTS.md`
  - render PNGs regenerated on different computers
- **Fixes:**
  - `.gitattributes` merges `LOG.md` with `merge=union` (local merges only).
  - `simulation/renders/` is no longer tracked; the README image moved to `docs/images/module.png`.
  - The root `AGENTS.md` has no status block.
  - Added "Working together (git)" to the root `AGENTS.md`.
- **After:** only 1 pull request needed a person: two software members editing the same line of
  `software/AGENTS.md`.
  - `LOG.md` conflicts on GitHub cleared with `git pull origin main`.
  - Hardware's mass change reached the software code and the sim on the next pull.
  - The stale-mesh check caught a new CAD file pushed without re-exported meshes.

## 2026-10-22 · Wesley (Claude Code)
- **Sync checks:** `hardware/spec.toml` now holds the chain's numbers, which the sim reads and the software should
  read. The sim refuses stale meshes (see `simulation/LOG.md`).
- Updated "Facts every area shares" and the cross-area table to point at the spec.
- Checked: `verify.py` ALL CHECKS PASSED; headless log byte-identical.

## 2026-10-22 · Wesley (Claude Code)
- Reorganised the flat `chain_sim` folder into `hardware/`, `software/`, `simulation/`, `sequences/`, `experiments/`
  and `docs/`, ready to share with the team on GitHub.
- Replaced the single `CLAUDE.md` with `AGENTS.md` files (root + one per area), so any coding agent can read them.
  - Claude Code reads `AGENTS.md` only when there's no `CLAUDE.md`, so don't add one back.
- Moved the professor's physics answers to `experiments/findings.md`.
- Added `experiments/README.md` (experiments A/B/C from the proposal) and `sequences/README.md` (the CSV format).
- Moved `proposal_grouping.png` to `docs/images/`.
- Added a "When a change crosses areas" table to `AGENTS.md`. Nothing syncs hardware → sim → software automatically
  yet, so that table is the safeguard.
- Checked:
  - `simulation/verify.py`: ALL CHECKS PASSED.
  - The headless `fold_sequence.csv` log is byte-identical to before the move.
