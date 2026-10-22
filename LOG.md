# Log: repo-wide and shared folders

Newest first. This file covers root files, `sequences/`, `experiments/` and `docs/`. Each area has its own `LOG.md`.
The format is in the root `AGENTS.md`.

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
