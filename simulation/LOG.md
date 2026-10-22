# Log: simulation

Newest first. The format is in the root `AGENTS.md`.

## 2026-10-22 · Wesley (Claude Code)
- `renders/` is no longer in git: `verify.py` rewrites it, and the PNGs differ between computers, which caused binary
  merge conflicts in a team rehearsal. The README image is now `../docs/images/module.png`.

## 2026-10-22 · Wesley (Claude Code)
- **original14 now comes from `../hardware/spec.toml`:** `make_chain.py` reads it for servo, pitch, masses, shell
  radius, ranges and module count. The panel's module default comes from it too.
- **Stale-mesh check:** `export_meshes.py` writes `meshes/SOURCE.txt` (CAD file name + SHA-256), and `verify.py`
  section 1 fails if the CAD has changed since.
- **`play.py` without `--model` builds the model fresh from the spec.** `chain.xml` is now git-ignored; the panel
  still writes it.
- Checked:
  - `verify.py` ALL CHECKS PASSED.
  - The XML for all 3 presets × slip ring is byte-identical to before, and so is the headless log.
  - Re-exported STLs unchanged.
  - Real launch: Play, 0 contacts.

## 2026-10-22 · Wesley (Claude Code)
- **Moved the sim into `simulation/`, with its own `.venv`.**
  - Sequences now come from `../sequences/` and the CAD from `../hardware/cad/`.
  - The defaults for `play.py --model/--log`, `make_chain.py -o` and `export_meshes.py` now resolve from this folder.
  - Code behaviour is unchanged.
  - `CLAUDE.md` became `AGENTS.md`.
- **Checked:**
  - `verify.py`: ALL CHECKS PASSED.
  - The headless `fold_sequence.csv` log, `chain.xml` and all four exported STLs are byte-identical to before.

## 2026-10-21 · Wesley (Claude Code)
- **Added Spin / Stop spin** to the control window, plus `spin` and `turned` readouts.
- **Checked:**
  - Tested Spin → Play → Stop spin through the real buttons.
  - Added the spin checks to `verify.py`.

## 2026-10-20 · Wesley (Claude Code)
- **Anti-wobble servo model:** gearbox friction, motor inertia (`armature`) and near-target braking (`kv`).
  - A 60° step now overshoots 1° instead of 9.5°.
- **Added `sequences/reorient_loop.csv`:** the zero-angular-momentum "falling cat" turn.

## 2026-10-18 · Wesley (Claude Code)
- **Removed the Home (straighten) key** after a report that Fn+arrows reset everything. Fn+Right is still
  unexplained; see open issue 1 in `AGENTS.md`.

## 2026-10-17 · Wesley (Claude Code)
- **First version:**
  - Meshes exported from `complete chain.3dm`.
  - `make_chain.py` with 3 presets.
  - `play.py` (viewer, sequences, logging) and `verify.py`.
  - The control window (`panel.py`, `Chain Sim.command`).
- **Checked:** tested clicking Play in the real GUI.
