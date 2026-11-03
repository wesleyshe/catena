# End effectors, phase 1: modules and setup files. Implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** an end-effector module can be fitted to either end (anchor, suction cup or thruster, by type only); a
setup file captures a whole run. Setups 1a–1c and 6 run from the shell and from the control window.

**Architecture:**
- `make_chain.build()` gets `ee1` and `ee_n` (the end past module N) type arguments and adds the bodies after the
  chain, so existing ids don't move.
- `play.py` stops assuming every actuator is a joint servo.
- A new stdlib-only `setup_file.py` reads and writes `simulation/setups/*.toml`. `play.py --setup` and the panel's
  Setup box both use it.
- Phase 1 types differ only in mass and colour. The grip disc, the clamp, suction and nozzles come in phases 2–4.

**Tech stack:** Python 3.11, MuJoCo 3.16, Tk, `tomllib`. The tests are `verify.py` `check()` lines; there's no
pytest here.

**Spec:** `docs/design/2026-10-28-end-effectors.md` (approved by Wesley 2026-10-29).

## Global constraints
- Both ends none → `build()` XML and the headless log are **byte-identical** to `fbe880f`. Every existing
  `verify.py` key number stands.
- `make_chain.py`, `setup_file.py` and `panel.py` stay **stdlib-only**.
- `m1` stays body id 1 with the free joint. Add end-effector bodies after the chain subtree (m2 stays id 3).
- Actuators: joint servos first, in CSV column order. `m1_yaw` is the first actuator when ee1 is fitted.
- Log columns: unchanged in phase 1.
- Placeholder masses live as constants at the top of `make_chain.py`, not in `hardware/spec.toml`:
  - anchor 200 g, suction 200 g, thruster 250 g
  - plus 55 g for ee1's twist servo, carried in ee1
- The 7 cm presets scale these by s³.
- End effector: a Ø139 mm sphere (`R_SHELL`), one pitch beyond its end module along the chain axis.
- `m1_yaw` range ±150° (`YAW_RANGE`); with `--slip-ring` it has no limit, like the other yaws.

## Review focus
1. **A typo'd setup key** (`graviy = 9.81`): loading fails, naming the key and the file. It isn't silently ignored.
   → Task 3.
2. **A setup whose sequence file doesn't exist:** loading fails, naming the path, before any viewer opens. → Task 3.
3. **A far-end table that doesn't match the module count** (`[ee12]` with `modules = 6`): loading fails and says
   the far end is `[ee6]`. → Task 3.
4. **A setup that spins about two axes, loaded in the panel** (it has one axis): the status line says only the largest
   one was kept. → Task 4.
5. **Save as, then Load:** the same fields come back, including a sequence outside `sequences/`. → Task 3 (round
   trip) and Task 4 (panel).

---

### Task 1: End-effector modules in `make_chain.py`

**Files:**
- Modify: `simulation/make_chain.py` (constants after line 63; `build()`)
- Test: `simulation/verify.py` (new section 9, `check_end_effectors()`)

**Interfaces:**
- Produces:
  - `EE_TYPES = ("none", "anchor", "suction", "thruster")`
  - `EE_MASS = dict(anchor=0.200, suction=0.200, thruster=0.250)` (kg)
  - `EE_SERVO_MASS = 0.055`
  - `EE_RGBA = {type: "r g b a"}`
  - `build(preset, n, slip_ring, gravity, air, cage, ee1="none", ee_n="none")`
- Names:
  - Bodies `ee1` and `ee{n}`.
  - Geoms `ee{k}_visual` (class visual, a sphere, coloured by type) and `ee{k}_sphere` (class collision, carries
    all the mass, `shellinertia="false"`).
  - ee1 joint and actuator `m1_yaw`.
- Placement:
  - ee1 is a child of m1 at `0 0 -pitch`, appended after `m1_arm`.
  - ee{n} is a child of m{n} at `0 0 +pitch`, with no joint.
- **m1_yaw axis `0 0 -1`.** Positive m1_yaw turns the m12 side +θ about the chain direction relative to ee1, the same
  sign as every other yaw (each turns its m12-side part +θ relative to the m1 side).
- With anything fitted, the height and `statistic extent` use `span = n + fitted ends` instead of `n`. The m1 y
  position is unchanged; with both ends fitted, the chain is still centred.

- [ ] **Step 1: Write `check_end_effectors()` (section 9, part 1) in `verify.py`.** Add it to `__main__` before
  `renders()`, and add it to the docstring.
  - **Every combination compiles.** For each `t1, tn` in `EE_TYPES²` (original14):
    - `nu == 22 + (t1 != "none")` and `njnt == 23 + (t1 != "none")`
    - `m.body("m1").id == 1` and `m.body("m2").id == 3`
    - total mass = no-ee mass + `EE_MASS[t1] + EE_MASS[tn]` + `EE_SERVO_MASS` if ee1 fitted
    - One `check` line for the 16 combinations. Every other preset with `ee1="anchor", ee_n="thruster"` compiles too.
  - **The modules are unchanged.** With `ee1="thruster", ee_n="anchor"`, every body, joint and actuator named in
    the no-ee model has equal mass, inertia, `ipos`, `pos` (except m1), joint axis and range, and actuator
    gain, bias and ranges.
  - **The m1_yaw axis.** At the zero pose, in m1's frame:
    - The joint axis is −Z.
    - ee1 sits at `(0, 0, -P)` and ee12 at `(0, 0, 12P)`.
    - Setting qpos `m1_yaw` to 30° leaves ee1's position unchanged and turns ee1's frame by `rot((0,0,1), -30°)`.
  - **Momentum.** Factor section 3's loop into `random_motion(m, d, T=12.0, amp=90, period=1.5) -> (drift, L, gross,
    stable)`; `check_momentum` keeps its output. Run it with `ee1="anchor", ee_n="thruster"`. The thresholds are the
    same as section 3's: drift < 1e-4 m, L < 1e-4·gross, stable.

- [ ] **Step 2: Run it and see it fail.**
  `cd simulation && .venv/bin/python -c "import verify as v; v.check_end_effectors()"` fails with a `TypeError`:
  `build()` has no `ee1`.

- [ ] **Step 3: Implement the constants and the `build()` arguments.** The `m1_yaw` actuator is skipped only when
  `ee1 == "none"`. Raise `ValueError` for a type not in `EE_TYPES`.

- [ ] **Step 4: Run section 9 again.** Expected: all `ok`.

- [ ] **Step 5: Check byte-identity against `fbe880f`.**
  - Make the worktree: `git worktree add "$SCRATCH/base" fbe880f`.
  - Write `build()` to a file from both trees, using `simulation/.venv/bin/python`.
  - Run `cmp`. Expected: no output.

---

### Task 2: `play.py` counts joint servos and modules

**Files:**
- Modify: `simulation/play.py`
- Test: `simulation/verify.py` section 9

**Interfaces:**
- Consumes: `build(..., ee1=, ee_n=)` from Task 1.
- Produces:
  - `modules(model) -> int`: bodies named `m<digits>`.
  - `joint_servos(model) -> int`: actuators with `trntype == mjTRN_JOINT`, which come first.
  - `Session.nj`, the joint-servo count.
  - `Session.summary() -> str`.
  - `Logger(path, nj)`.
- `read_sequence` returns keys of width `nj`, and uses `modules(model)` for headerless files.
- Writes go to `ctrl[:nj]`: in `advance` and in `straighten`.
- Max torque is `actuator_force[:nj]`, in both the `Logger` and `readouts`.
- `Session.mid = m{max(1, modules // 2)}`, which is the same module as before (`(nu+2)//4 = n//2`).
- `run_viewer` uses `modules(model)`.
- The headless run prints `summary()` after "ran X s":
  `turned {turned:+.1f} deg, IMU turn {imu_turn:.1f} deg, spin {spin_rate:+.1f} deg/s, COM moved {mm:.3f} mm`.

- [ ] **Step 1: Add the section 9 check.** With ee1 fitted:
  - `read_sequence` of a temporary CSV with `m1_yaw = 20` puts `radians(20)` in column 0.
  - `Session.advance` drives `m1_yaw`: after 1 s, qpos `m1_yaw` is within 1° of 20°.
  - Without ee1, the same file gives zeros. The existing warning still prints.
- [ ] **Step 2: Run it and see it fail.** Expected: today's `n = (nu+2)//2` happens to give 12 with 23 actuators, so
  it may pass. The failing part is that `Session.nj` is missing (`AttributeError`).
- [ ] **Step 3: Implement it as in Interfaces.**
- [ ] **Step 4: Run section 9.** Expected: `ok`.
- [ ] **Step 5: Check the headless logs are byte-identical against `fbe880f`.** Compare two runs:
  - `--seq ../sequences/fold_sequence.csv --headless --log F`
  - `--seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air --cage --log F`

  Expected: `cmp` is silent for both.

---

### Task 3: Setup files: `setup_file.py`, `play.py --setup`, example setups

**Files:**
- Create: `simulation/setup_file.py`
- Create: `simulation/setups/1a_turn.toml`, `1b_spin_0.5.toml`, `1b_spin_1.toml`, `1c_roll.toml`, `6_skater.toml`
- Modify: `simulation/play.py` (CLI)
- Test: `simulation/verify.py` section 9

**Interfaces:**
- Consumes: `EE_TYPES`, `N_MODULES`, `PRESETS`, `build` from `make_chain`.
- Produces (`setup_file.py`, stdlib only):
  - `SETUP_DIR`, `SEQ_DIR`
  - `DEFAULTS`: sequence `""`, move_s 3.0, hold_s 1.0, speed 1.0, gravity_m_s2 0.0, spin_deg_s [0, 0, 0],
    air False, cage False, preset "original14", modules `N_MODULES`, slip_ring False, ee1 "none", ee_n "none"
  - `load(path) -> dict`: flat, with `ee1` and `ee_n` as type strings. `sequence` is an absolute path, or "".
  - `about(path) -> str`: the first `#` line, without the `#`.
  - `save(path, setup, about)`
  - `build_args(setup) -> dict`: the keyword arguments for `build()`.
- **File layout:** the keys above at the top level, except the ends:
  - The ends go in `[ee1]` and `[ee{modules}]` tables with a `type` key. A missing table means none.
  - A relative `sequence` is relative to `SEQ_DIR`.
- **`load` raises `ValueError`, naming the file, for:**
  - an unknown key
  - a type not in `EE_TYPES`
  - an `[eeK]` table with K not 1 or `modules` (the message names the right table)
  - a sequence file that doesn't exist
- **`save` writes TOML by hand:**
  - the about line, then every top-level key, then a table for each fitted end only
  - strings via `json.dumps`, floats via `repr`, booleans as `true`/`false`
  - a sequence inside `SEQ_DIR` saved as its basename
- **`play.py --setup FILE`:**
  - Parse once, then call `ap.set_defaults(...)` from the setup and parse again, so explicit flags still win.
  - The setup sets seq, move, hold, speed and gravity, and air and cage (store_true, so a flag can only add).
  - It sets spin only if it's non-zero, which keeps the 1a log identical to the plain-flags run.
  - Without `--model`, the model comes from `build(**build_args(setup))`.

- [ ] **Step 1: Write the setup checks in section 9.** All of them use temporary files:
  - an unknown key, a missing sequence, and `[ee12]` with `modules = 6` each raise `ValueError` whose message
    contains the key, the path, or `[ee6]`
  - a `save` → `load` round trip with ee1 anchor, ee_n thruster, spin [0, 0, 0.5], an absolute sequence outside
    `SEQ_DIR`, and modules 8 gives an equal dict
  - every `setups/*.toml` loads, compiles through `build_args`, and its sequence reads with `read_sequence`
- [ ] **Step 2: Run it and see it fail.** Expected: `ModuleNotFoundError: setup_file`.
- [ ] **Step 3: Implement `setup_file.py` and the `--setup` CLI.** Write the five setups in `save()`'s layout:

  | Setup | about line | Keys that aren't defaults |
  |---|---|---|
  | `1a_turn` | turns about 30° with no push | `reorient_loop.csv`, move 1.0, hold 0.25 (flight timing, 20 s) |
  | `1b_spin_0.5` | let go spinning at 0.5°/s, half the turn is gone | as 1a, spin [0, 0, 0.5] |
  | `1b_spin_1` | let go spinning at 1°/s, the turn is cancelled | as 1a, spin [0, 0, 1] |
  | `1c_roll` | rolls about its own length, about 63° | `roll_loop.csv`, speed 2 |
  | `6_skater` | spins faster folded, like a skater | `fold_sequence.csv`, spin [0, 0, 20] |

  1a and 1b use flight timing, not ×4 as the design table said, because the −15°/0° release-spin numbers in
  `findings.md` were measured at that timing.
- [ ] **Step 4: Run section 9.** Expected: `ok`.
- [ ] **Step 5: Run each setup headless and compare with `findings.md`.**
  `.venv/bin/python play.py --setup setups/<name>.toml --headless --log ""`

  | Setup | Expected |
  |---|---|
  | 1a | turned −30.3° |
  | 1b 0.5 | turned −15.2° |
  | 1b 1 | turned 0.0° |
  | 1c | IMU turn 63.4° |
  | 6 | spin +20.0 at the end |
  | all | COM moved 0.000 mm |

  Also: 1a's log is `cmp`-identical to `--seq ../sequences/reorient_loop.csv --move 1 --hold 0.25 --headless`.

---

### Task 4: Control window: Setup box, End effectors box, spin at Play

**Files:**
- Modify: `simulation/panel.py`
- Test: `simulation/verify.py` section 9 (the pure helper), plus a scripted run of the real panel

**Interfaces:**
- Consumes: `setup_file.load/about/save/build_args/SETUP_DIR/SEQ_DIR`, `EE_TYPES`.
  - The panel imports `SEQ_DIR` from `setup_file` instead of defining its own.
- Produces: `spin_from_vector(v) -> (rate, axis_label, warning)` in `panel.py`. It's a module-level function, and
  `axis_label` is one of `SPIN_AXES`. For two or more non-zero components it keeps the largest, and `warning` names
  the dropped ones. For all-zero it returns `(0.0, SPIN_AXES[0], "")`.
- **Setup box, at the top:**
  - a combobox of `setups/*.toml` that re-reads the folder when opened
  - a label showing the selected setup's about line
  - **Load:** fills every field (Model, End effectors, Fold sequence, World gravity, Spin), then `open_viewer()`
  - **Save as...:** `filedialog.asksaveasfilename` in `SETUP_DIR`, plus `simpledialog.askstring` for the about line,
    prefilled
  - errors go to the status line
- **End effectors box, after Model:** two Type comboboxes (`EE_TYPES`), "past m1 (twist servo)" and "past the last
  module". They're applied by Rebuild.
- **Spin box:** a new **at Play** tick box. Play sends `spin {rate, axis}` before `play` when it's ticked. Load ticks
  it when the setup spins; Save as writes `spin_deg_s` only when it's ticked.
- **The protocol doesn't change.**

- [ ] **Step 1: Write the `spin_from_vector` checks in section 9.**
  - `[0, 0, 0.5]` gives `(0.5, "vertical (z)", "")`.
  - `[0, 2, 0.5]` gives `(2, "along chain (y)", <a warning mentioning z>)`.
  - `[0, 0, 0]` gives `(0.0, "vertical (z)", "")`.

  Import `panel` in verify. It's stdlib-only and builds no window at import.
- [ ] **Step 2: Run it and see it fail.** Expected: `AttributeError`, no `spin_from_vector`.
- [ ] **Step 3: Implement the boxes and the helper.**
- [ ] **Step 4: Run section 9.** Expected: `ok`.
- [ ] **Step 5: Run the real window, with scripted buttons.**
  - Check `ps` for an open Chain Sim first, and start our own instance.
  - Pick `6_skater`, invoke **Load**, wait for the viewer to connect, then invoke **Play**.
  - The readouts show `spin +20.0 deg/s` and `fold_sequence.csv`.
  - Then set ee1 to thruster, invoke Rebuild, and check that `chain.xml` has `ee1`.
  - Save as to a scratch file (patch the dialogs) and load it back.

---

### Task 5: Docs, full check, commit

**Files:**
- Modify: `simulation/AGENTS.md` (status, folder map, contracts, verification), `simulation/README.md` (commands,
  control-window table), `simulation/LOG.md`
- Modify: root `AGENTS.md` (shared facts: m1_yaw is live when ee1 is fitted, 23 joints), `sequences/README.md`
  (columns with no servo), root `LOG.md` (the shared-fact change)
- Modify: the design doc's status line

- [ ] **Step 1: Update the docs** per the root `AGENTS.md` rules: edit the lines that are no longer true, no
  "update:" notes.
- [ ] **Step 2: Run `.venv/bin/python verify.py`.** Expected: `ALL CHECKS PASSED`, 9 sections, the old key numbers
  unchanged.
- [ ] **Step 3: Clean up:** `__pycache__/`, `MUJOCO_LOG.TXT`, the scratch worktree (`git worktree remove`).
- [ ] **Step 4: Commit** on `sim/end-effectors`: the design doc, this plan, the code, the setups and the docs, in one
  commit. Don't push.
