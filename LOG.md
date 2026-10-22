# Log: repo-wide and shared folders

Newest first. This file covers root files, `sequences/`, `experiments/` and `docs/`. Each area has its own `LOG.md`.
The format is in the root `AGENTS.md`.

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
