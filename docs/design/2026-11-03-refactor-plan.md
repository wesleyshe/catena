# Refactoring the simulation: plan

**Date:** 2026-11-03 · Wesley (Claude Code) · branch `sim/refactor-plan`

**Status:** a plan only. No code has changed. Nothing here starts until Wesley says go.

**Goal:** the same sim, in files that each do one thing, so a teammate or an agent can find and change one part
without reading 1,000 lines first. Nothing anyone sees or relies on changes: the commands, the control window, the
setups, the logs and the numbers stay the same.

## Where it stands (main at `69442e3`, measured 2026-11-03)
- `verify.py`: ALL CHECKS PASSED (71 checks in 15 sections, about 100 s when nothing else is running).
- All 21 setups fly headless. Their logs and last lines are the baseline for the checks in section 4.
- `make_chain.build()` for 47 models (every setup, preset, slip ring, end-effector pair, `--box`) takes 0.09 s.
- **Runs repeat exactly.** Two runs in a row gave byte-identical logs for the fold sequence and for all 21 setups, and
  the same verify output. So "byte-identical" works as a test for every setup, not only the fold sequence.

## 1. What has grown messy

### `play.py` (1,066 lines) is five things in one file
- **The fold-sequence format:** `read_sequence`, `csv_rows`, `read_ee`, `target`.
- **The flight timeline:** `Parabola`, `hermite`, `pitch_matrix`. This is just maths of time, with no MuJoCo in it.
- **The run itself, `Session` (573 lines).** It holds:
  - the state, the start pose, playing a sequence, and spin and turn tracking;
  - the end effectors' behaviour (about 95 lines: `grip`, `nozzle`, `clamp`, `capture`, `update_ends`);
  - the flight box at run time (about 195 lines: the strings, the gap to the box, the step before and after,
    the landing);
  - the log columns, the command dispatch, and every line of text in the readouts, the summary and the banner.
- **The viewer window:** `run_viewer` (107 lines), the countdown queue `Commands`, the keys, screenshots and the socket.
- **The command line.**

Specific problems:
- End effectors are split in two. `End` holds the state and `Session` holds the behaviour, and `place`, `reset` and
  `flying` also reach into them.
- The flight's state (`parabola`, `released`, `touch`, `landing`, …) only exists after `fly()`. `summary()` has to
  check `getattr(self, "parabola", None)`.
- Three names mean "is there a box": `self.flight` (the setup's table), `self.box` (the box's geoms) and `self.cabin`
  (its body).
- The plane's pitch rate is worked out by finite differences in three places (`flying`, `flight_readouts`, verify), each
  with a different step. The one in `flying` drives the physics, so its expression must stay exactly as it is.
- Building a model from a setup is written out three times, in `play.py`, `verify.py` and `stall_sweep.py`: `build(...)`,
  then `.replace('meshdir="meshes"', ...)`, then `from_xml_string`.

### `make_chain.py` (447 lines)
- **`build()` is 232 lines.** In order, it writes:
  - the options and defaults, and the world;
  - the whole flight box: acrylic, net, cabin floor, camera, plates and rods on the cabin;
  - the chain;
  - the end effectors: bodies, welds, cups, nozzles;
  - the actuators, contact pairs, strings and sensors.
- **Forty constants sit at the top.** The physics people tune (`PRESETS`, `GEAR_FRICTION`) is mixed in with the box's
  aluminium, mesh and door colours. `box_looks` is 32 lines of looks in the middle of the physics.
- **The cabin is built in pieces.** Its element is created early and filled in three places: the box, then plates and
  rods, then the strings' end sites. It's added to the world after the chain, so m1 stays body 1. That's correct, but
  hard to follow.

### `verify.py` (796 lines)
- 15 sections in one file, defined out of order: 8 (renders) sits between 7 and 9, and 14 comes after 15.
- Section 9, "end effectors", also holds every setup-file check: bad keys, Save as then Load, every setup loads, the
  demo buttons, the sequences' first lines. That's 60 lines that have nothing to do with end effectors.
- Section 7 adds a fake `weighed` preset to `make_chain.PRESETS` and deletes it afterwards. If the build failed in
  between, the fake preset would stay.
- There's no way to run one section, so every small change costs about 100 s.

### `panel.py` (543 lines)
- `Panel.__init__` is 167 lines that build all nine sections.
- `play()` and `fly()` repeat how a take starts: speed, playback, the countdown box's fallback and the spin vector.
  `current()` repeats the spin vector a third time.
- Otherwise it's fine. `EndBox` is tidy.

### Smaller things
- `setup_file.RAMPS_S = 7.0` is a hidden copy of `Parabola`'s ramps (3 s + 4 s). Change one without the other, and
  setups get checked against the wrong minimum.
- `export_meshes.PITCH = 140` copies the spec's `pitch_mm`. It's the CAD file's spacing, so it should match the spec, but
  nothing checks that it does.
- `HERE` is defined in 9 files, and the sequences folder in two (`setup_file.SEQ_DIR`, `verify.FOLD_SEQ`).
- `simulation/AGENTS.md` is 259 lines, over the root `AGENTS.md`'s limit of about 250.

### What's fine and stays as it is
`setup_file.py` (only its imports change), `viewer_link.py`, `export_meshes.py`, `inspect_3dm.py`, `stall_sweep.py` and
`pole_climb_design.py` (imports only), `Chain Sim.command`, `setups/`, `meshes/`, `requirements.txt`.

## 2. The proposed structure
The rules it follows:
- **Every command stays the same:** the same scripts, the same flags, the double-click. Only the files behind them
  change.
- **One file per idea,** each under about 350 lines.
- **Each number people edit sits at the top of the file that uses it.** Most stay where they are now (see the table
  below).
- **Two sides, as now.** The stdlib side is the window and everything it imports, so it runs without MuJoCo. The MuJoCo
  side is the run.
- **`Session` stays the one object everything drives,** and keeps every name in the contract.
- **Flat folder, no package.** Scripts import each other by name, as they do now, and every file name in the docs stays
  short.

### Files after
| File | Side | What it holds | ~Lines |
|---|---|---|---|
| `panel.py` | stdlib | The control window, one method per section | 520 |
| `setup_file.py` | stdlib | Setups: `load`, `save`, `summary`, `DEMOS` (unchanged) | 335 |
| `viewer_link.py` | stdlib | Unchanged | 75 |
| `spec.py` (new) | stdlib | Reads `../hardware/spec.toml` once; the folders (sequences, setups, meshes, logs, screenshots); `v()` for numbers in the XML | 30 |
| `make_chain.py` | stdlib | `PRESETS` and the chain's numbers at the top; `build()`: the world, the chain, plates and the rod, then the parts below; the command line | 230 |
| `make_ends.py` (new) | stdlib | The end effectors in the model: `EE_TYPES`, `EE_MASS`, `EE_DEFAULTS`, the clamp's weld, cups, nozzles, `FACINGS`, `attach_point`, `leading_end` | 110 |
| `make_box.py` (new) | stdlib | The flight box in the model: acrylic, net, cabin floor, camera, the strings, its look; its numbers at the top | 170 |
| `play.py` | MuJoCo | The command line only: flags, build the model, a `Session`, then headless or the viewer | 90 |
| `session.py` (new) | MuJoCo | `Session` and `Logger`: state, start pose, playing a sequence, the step, spin and turn, commands, log columns, readouts and summary; `load_model()` | 330 |
| `sequence.py` (new) | MuJoCo | The fold-sequence format: read the angles and end-effector columns, ease between rows | 80 |
| `ends.py` (new) | MuJoCo | `End`, one end effector at run time: hold, let go, grab on touch, nozzles, loads, its readout line | 140 |
| `flight.py` (new) | MuJoCo | `Parabola` (the g and the pitch over time) and `Flight` (the box at run time: strings, gap, the step before and after, the landing, its readouts and banner) | 280 |
| `viewer.py` (new) | MuJoCo | The viewer window: loop, keys, the countdown (`Commands`), camera, screenshots, readouts to the panel | 180 |
| `verify.py` | both | Runs `checks/` in order and prints ALL CHECKS PASSED; `verify.py 15` runs one section | 40 |
| `checks/` (new) | both | The 15 sections, one file per subject (model, momentum, flight, end effectors, setups, …) | 780 |

### How the pieces depend on each other
```
stdlib side                                         MuJoCo side
spec.py --> make_ends.py --+
        --> make_box.py ---+--> make_chain.py       sequence.py --+
                                   |                ends.py ------+--> session.py --> viewer.py --> play.py
                                   v                flight.py ----+
                            setup_file.py --> panel.py  ==(socket)==>  play.py --panel  (under mjpython)
verify.py --> checks/ --> everything
```
An arrow points from a file to the files that import it. The MuJoCo side also imports the stdlib side (for example,
`ends.py` reads `make_ends.NOZZLE_RGBA`). Nothing on the stdlib side imports the MuJoCo side; a check enforces that
(section 4, G).

### Interfaces between them
- **`make_chain.build(preset, n, slip_ring, gravity, air, ee1, ee_n, rod, flight)` is unchanged.** Inside, it calls:
  - `make_box.cabin(box_m, s, floor_friction)`: the cabin element (box, net, cabin floor, look, camera), not yet in the
    world;
  - `make_box.strings(root, cabin, strings, n, pitch, box_m)`: the tendons and their end sites;
  - `make_ends.bodies(root, world, first, last, ends, n, s, pitch, yaw_limits)`: adds `ee1` / `ee{N}` and returns their
    actuators, which `build()` writes after the joint servos;
  - `make_ends.cup_pairs(contact, ends, n, fixed_geoms)`.

  `held_height` and `string_ends` move to `make_box`. The net's rolling friction is passed in from `make_chain`'s
  `ROLLING_RESISTANCE` and `R_SHELL`, not copied.
- **`sequence.py`:** `read_sequence`, `read_ee`, `target` and `EE_COLUMN` keep their names and signatures.
- **`End(model, data, name, world)`.** Here `world` says which geoms are fixed and where the cabin is. Its methods are
  `grip(on)`, `nozzle(k, throttle)`, `clamp(shift)`, `update()` and `readout()`.
- **`Flight(session)`.** `Session` makes one when the model has a cabin, and it uses the session's chain: `spin`,
  `inertia`, the shells, the ends, `play`. Its methods are `start(...)` (what `fly` does now), `let_go()`, `stop()`,
  `before_step()`, `after_step()`, `frame()`, `com()`, `gap()`, `tension()`, `strings_on(on)`, `result()`,
  `readouts()` and `banner()`.
- **`Session(model, data, log_path=None, setup=None)` is unchanged, and every name in the contract stays on it.**
  - These pass straight to `Flight` or `End`, one line each: `fly`, `let_go`, `grip`, `nozzle`, `box_frame`,
    `box_com`, `box_gap`, `strings_on`, `tension`, `flight_result`, `banner`.
  - The flight's state reads through as properties: `fly_t`, `parabola`, `released`, `release_s`, `release_spin`,
    `liftoff`, `touch`, `gap_min`, `landing`, `dip`, `manual`, `com0_box`.
- **`session.load_model(**build_args)`** returns `(model, data)` with the meshes' path set. `play.py`, `checks/` and
  `stall_sweep.py` use it in place of their three copies.
- **`viewer.run(session, panel_port)`** (today `play.run_viewer`) and `viewer.Commands`.
- **`play.py` re-exports** `Session`, `Parabola`, `Commands`, `read_sequence`, `target`, `joint_servos`, `modules` and
  `LOG_DT`, so `from play import Session` keeps working for anyone's own scripts.

### Where the numbers people edit end up
| Numbers | Today | After |
|---|---|---|
| `PRESETS`; `PITCH_RANGE` … `ROLLING_RESISTANCE`, `CHAIN_ROLL`, `CORE_BOX`, `ARM_BOX`, the chain's colours; `PLATE`, `DECK_T`, `ROD_MM` | top of `make_chain.py` | top of `make_chain.py` (unchanged) |
| `EE_MASS`, `EE_SERVO_MASS`, `EE_DEFAULTS`, `TIP`, `CLAMP_SOLREF`, `CUP_T`, `CUP_MARGIN`, `NOZZLES`, their colours | top of `make_chain.py` | top of `make_ends.py` |
| `ACRYLIC_T`, `FRAME_MM`, `NET_FRAME_MM`, `MESH_MM`, the box's colours, `STRING_SOLREF`, `STRING_FAN`, `NET_IMPEDANCE`, `NET_DAMPING` | top of `make_chain.py` | top of `make_box.py` |
| `CAPTURE`, `PULL_IN`, `SOFT_SOLREF`, `LOAD_TAU`, `CUP_TOUCH` | top of `play.py` | top of `ends.py` |
| `Parabola`'s `LEVEL_S`, `UP_S`, `DOWN_S`; `TOUCH` | `play.py` | top of `flight.py` |
| `LOG_DT` | top of `play.py` | top of `session.py` |
| `STEP_DEG`, `KEYS`, the selected module's colour, `WINDOW_NAME` | top of `play.py` | top of `viewer.py` |
| `COUNTDOWN_S`, `ROD`, `FIRST_SEQ` | top of `panel.py` | unchanged |
| The real chain's numbers | `hardware/spec.toml` | unchanged |

## 3. The contracts, and how each is kept
| Contract (`simulation/AGENTS.md`) | How it's kept | Checked by |
|---|---|---|
| `verify.py` ends with ALL CHECKS PASSED; the key numbers | Sections move word for word | A: verify's output compared line by line with the baseline |
| The headless `fold_sequence` log is byte-identical | Physics code moves without any change to its arithmetic | B: the log's SHA-256 |
| All 21 setups fly the same | The same | C: each setup's log SHA-256 (stronger than comparing results) and its last lines |
| Model names; m1 is body 1 | `make_*` write the same elements in the same order | D: SHA-256 of `build()` for 47 models, byte-identical XML |
| Log columns | `log_names` / `log_columns` unchanged, the ends in the same order | B and C (the hashes include the header) |
| Panel ↔ viewer protocol | `Session.command` and `Commands` move unchanged; the panel sends the same lines | E, plus verify section 14 |
| `panel.py`, `setup_file.py`, `make_chain.py` stay stdlib-only | `spec.py`, `make_ends.py` and `make_box.py` are stdlib-only too | G |
| `play.py` API used elsewhere | The names stay on `Session`; `play.py` re-exports them | `verify.py` and `stall_sweep.py` run unchanged until step 8 moves their imports |
| Setup file format | `setup_file.py` unchanged | verify's setup checks |
| Paths resolve from the script folder | `spec.py`'s folders | Run `play.py` once from the repo root |
| Coupled physics settings | Untouched | A |
| The numbers people edit stay at the top of their files | The table above | Read the top of each file; README and AGENTS.md say where each number is |

## 4. The checks
Run A–D and G after every step; they take about 3 minutes together on this Mac. E and F apply where a step says so.
- **A. `verify.py`:** ALL CHECKS PASSED, and its printout compared with the baseline's (only the times may differ).
- **B. The fold log:** `play.py --seq ../sequences/fold_sequence.csv --headless --log FILE`, compared by SHA-256.
- **C. The 21 setups headless,** 12 at a time (about 2 minutes). Each log compared by SHA-256, and each run's printout
  line by line.
- **D. `build()` XML** for 47 models: every setup, each preset with and without the slip ring, the 16 end-effector
  pairs, `--box`, and gravity with air and 8 modules. Takes 0.1 s, and points straight at the model a mistake broke.
- **E. Panel ↔ viewer** (steps that touch `panel.py` or the viewer). The real control window is driven by script, with
  its link to the viewer replaced by a recorder. For every button, it records the command lines sent and the
  `panel_setup.toml` each demo writes, and compares them with the baseline.
- **F. The real app** (steps 9 and 10, and at the end). Double-click `Chain Sim.command`, then try a demo button,
  Start the parabola, Let go, Play, the camera tick, Screenshot, and the keys.
- **G. Stdlib only:** import `panel.py` (and through it everything on the stdlib side) with `mujoco` and `numpy`
  blocked.

Step 0 writes A–E and G as one script: `checks/same_as_before.py save DIR`, then `compare DIR`. The baseline stays on
the computer that made it: floating point can differ between computers, so the hashes can't go in CI.

## 5. Migration order
Each step is one small change, and it passes A–D and G (plus E and F where noted) before the next one starts. Commits
come at the end of each phase, after verify passes. Nothing is pushed until Wesley asks.

### Phase 0: the safety net
**Step 0.** Write `checks/same_as_before.py` and the stdlib check. Save the baseline on `main`, then run `compare`
against it twice.
- *Check:* both compares pass. This repeats what was measured on 2026-11-03.
- *Could go wrong:* nothing in the sim changes. The recorder in E must never open a real viewer.

### Phase 1: the model in three files
**Step 1, `spec.py`.** Move the spec reading and the folders out of `make_chain.py` and `setup_file.py`.
`make_chain` keeps its names (`HW`, `SERVO`, `MASS`, `FLIGHT`, `SPEC_FILE`) by importing them.
- *Check:* D, G, A.
- *Could go wrong:* an import cycle; a path that resolves differently when run from another folder.

**Step 2, `make_ends.py`.** Move the end effectors' numbers, `attach_point`, `leading_end` and `level_axis`, and the
XML for `ee1` / `ee{N}`: bodies, tip, hold and weld, cup, nozzles and the cup contact pairs. Update the imports in
`play.py`, `setup_file.py`, `panel.py`, `verify.py` and `pole_climb_design.py`.
- *Check:* D for all 16 end pairs and every setup; G; A, sections 9–13.
- *Could go wrong:* the order of elements:
  - the `equality` block is created with the first anchor;
  - the actuators must still follow the joint servos;
  - the cup pairs list the world's and cabin's geoms as they are at that point;
  - `ee{N}`'s parent is the last module's body, which is the loop variable `body` today.

**Step 3, `make_box.py`.** Move the cabin (acrylic, net, cabin floor, look, camera), `held_height`, `string_ends`,
`net_contact`, the strings' tendons and end sites, and their numbers.
- *Check:* D for every flight setup and `--box`; A, sections 7 and 15.
- *Could go wrong:* the cabin's children must keep their order: box, look, camera, then plates and rods, then the
  strings' end sites.

→ **Commit:** "the model in three files".

### Phase 2: the run in five files
**Step 4, `sequence.py`.** Move `read_sequence`, `csv_rows`, `read_ee`, `target` and `EE_COLUMN`.
- *Check:* A–C.
- *Could go wrong:* little. Warnings must still print in the same order.

**Step 5, `flight.py`, part 1.** Move `Parabola`, `hermite` and `pitch_matrix`. Add a check that
`setup_file.RAMPS_S` equals `Parabola.UP_S + DOWN_S`.
- *Check:* A–C.
- *Could go wrong:* little.

**Step 6, `ends.py`.** `End` gets its behaviour: `grip`, `nozzle`, `clamp`, `capture`, `update` and `readout`.
`Session.grip`, `nozzle`, `nozzles_off` and `ee_command` become one line each.
- *Check:* A–C. The end-effector setups (2*, 3*, 4*, 5b) are the ones that matter.
- *Could go wrong:*
  - the order of things within a step: anchors grab, the pull-in ends, then the loads are smoothed;
  - a clamp in the box remembers its hold in the box's frame;
  - the order of `ends`, which sets the log columns.

**Step 7, `flight.py`, part 2: `Flight`.** `Session`'s flight methods move into it, and `Session` passes the contract
names through.
- *Check:* A–C, so all 21 setups byte-identical. Also a headless flight by hand (`fly(manual=True)`, then `let_go`,
  then `play`), compared before and after.
- *Could go wrong:* the most of any step.
  - `fly_t` adds `LOG_DT` each step, so the order of operations must stay the same.
  - The pitch rate that turns the cabin (±5 ms) must keep its exact expression.
  - `let_go` calls `spin`, and the parabola's step starts the sequence.
  - `reset` ties the strings back on and measures the gap and `com0_box` again.
  - State that only existed after `fly()` needs a starting value that doesn't change `summary()`.
- *Stop rule:* if `Flight` turns out to need more than a handful of `Session`'s insides, keep the flight methods as one
  block in `session.py` instead, and move on.

**Step 8, `session.py`.** `Session`, `Logger` and `load_model` move out of `play.py`. `play.py` keeps the command
line and the re-exports. Point `verify.py` and `stall_sweep.py` at the new files.
- *Check:* A–C, and run `play.py` once from the repo root.
- *Could go wrong:* relative paths.

→ **Commit:** "the run in five files".

### Phase 3: the windows
**Step 9, `viewer.py`.** Move `run_viewer`, `Commands`, `connect_panel`, `screenshot` and `KEYS`.
- *Check:* A (section 14), E, and F: a demo button, the countdown, Start the parabola, Let go, Play, PageUp and the
  arrows, Space, Screenshot, and the camera tick.
- *Could go wrong:*
  - mjpython needs the main thread;
  - GLFW's window-name hint must still be set before `launch_passive`;
  - the README says `KEYS` is at the top of `play.py`.

**Step 10, tidy `panel.py`.** One method per section; one `_start_take()` shared by `play` and `fly`; one helper for the
spin vector.
- *Check:* E (every button sends exactly what it did, and each demo writes the same `panel_setup.toml`), G, and F (the
  window looks the same, with screenshots side by side).
- *Could go wrong:* Tk lays out widgets in the order they're made, so a section built in a different order moves on
  screen.

→ **Commit:** "the windows".

### Phase 4: the checks
**Step 11, `verify.py` → `checks/`.** Move the sections word for word, in their printed order, with `check()` and
`failures` shared. Then:
- move the setup-file checks out of section 9 into a new section 16, "setup files". This is the only change to verify's
  printout;
- add `verify.py 15` to run one section;
- fix section 7's borrowed preset, with `try`/`finally` or a `build()` argument.
- *Check:* A's printout identical up to the move to section 16; the `verify` check on GitHub goes green on the pull
  request.
- *Could go wrong:* a section that relied on another's leftovers; the osmesa renders on CI.

→ **Commit.**

### Phase 5: the docs
**Step 12.** Update:
- the README's Files and Commands tables;
- `AGENTS.md`'s folder map, contracts (the file names) and "How the pieces connect";
- where each number lives;
- the stale lines in section 6.

Then bring `AGENTS.md` under 250 lines.
- *Check:* every file and function name the docs mention exists (grep), and the README's commands run.

→ **Commit.** Pull request when Wesley asks.

Each phase is about one session's work.

## 6. The rest of the repo, briefly
- **`hardware/spec.toml`** is fine as the one source for the AX-12A chain.
  - The flight build's numbers (`feetech85`: STS3032 at 0.44 N·m and 11.6 rad/s, plus the scaled masses) are typed into
    `make_chain.PRESETS`. That's against the root rule that numbers live in the spec.
  - When the STS3032 CAD lands, hardware could add them to the spec, for example as a `[flight_build]` table, and
    `feetech85` could read them from there. That's hardware's call, and not part of this refactor.
- **`export_meshes.PITCH`:** read it from the spec, or have verify check that the two agree. A small fix.
- **`sequences/`:** the format is clean, and there's one stale row (below). Verify checks that each CSV has a first
  line. It could also check that the README's tables list every CSV.
- **`experiments/`:** the numbers in `findings.md` come from the sim. A pure refactor leaves them standing, and check C
  proves it. There's one stale line (below).
- **`docs/`:** two PDFs sit there untracked:
  - the Novespace ITF, which `spec.toml` cites;
  - an example safety data package from another team.

  The repo is public, so neither should be committed unless they're ours to share. Suggestion: leave both out (add them
  to `.gitignore`) and say in `spec.toml` where to get the ITF.
- **`software/`:** empty; nothing to do.
- **Stale lines in the docs.** Fix them in step 12, or now as a separate docs-only change:
  1. `setups/1d_flight_turn.toml` appears in `simulation/AGENTS.md:41`, `simulation/README.md:118` and `play.py:10`. It
     was removed on 2026-11-03: 1d is now `1d_roll`, and the flight example is `1a_turn`.
  2. `simulation/README.md:311` says "All three presets compile", but there are four.
  3. `simulation/README.md:228` says `stall_sweep.py` works "from the 2b setups". It builds its own 14 cm setups in code.
  4. `simulation/README.md:229` says `pole_climb_design.py` "writes its sequence and setup". It writes only the
     sequence.
  5. `simulation/README.md:352` says "see open issue 9". It's issue 10.
  6. `simulation/README.md:5` says the chain floats about 1 m above a floor, but every setup now flies in the box.
  7. `sequences/README.md:31` (`inchworm_walk`) says "the box's lid in 4a". 4a stands on a deck over the net.
  8. `experiments/README.md:65` lists "suction cups and thrusters too (setups `3a_*`, `4a`, `4b`, `5a`)". 5a is the pole
     climb, which has no end effectors, and 5b's rail crawl uses anchors.
  9. `README.md:14,17` describes `simulation/` as "zero-g folding, spin, gravity, logging", which leaves out the flight
     box and end effectors. It describes `docs/` as "Proposal and course syllabus", which leaves out the design notes.
  10. `hardware/AGENTS.md:113` says "Outcome (2026-10-22): … about 80 mm". The Ø85 mm decision of 2026-11-03, in the
      status above it, supersedes that.
  11. `simulation/AGENTS.md` is over its length limit (259 lines). Its contract line "`verify.py` uses `Session` (…)"
      leaves out `Commands`, `Parabola`, `LOG_DT` and `joint_servos`.
  12. `simulation/README.md:448` lists the real-GUI tests up to 2026-10-21. The 2026-11-03 window tests are missing.

## 7. Decisions for Wesley
1. **Go ahead with all of it, or part?** My pick: phases 0–2 first (the model and the run), since that's where most of
   the mess is. Then phases 3–5.
2. **Commit `checks/same_as_before.py`, so later refactors can reuse it?** My pick: yes.
3. **Add `verify.py 15` to run one section?** My pick: yes. It's what makes small steps cheap.
4. **Fix the stale docs now, as a separate docs-only change, rather than in step 12?** My pick: now, because they
   mislead today.
5. **Keep the two untracked PDFs out of git?** My pick: yes.

## As built (2026-11-03)
Wesley said go ("your plan looks good. Carry on"), and all five phases are done: commits `2e49a22` (0–1), `075697b` (2),
`603dabb` (3), `d06771b` (4) and the docs after them. Each phase compared 48 of 48 the same as the baseline
(`checks/same_as_before.py`). Where it differs from the plan above:
- **The flight's methods moved as a block, not into a `Flight` object.** A `Flight` would have needed most of
  `Session`'s insides (spin, inertia, play, the ends, the shells, the sequence's state), which tripped the stop rule in
  step 7. They're in `flight.py` as `FlightBox`, which `Session` inherits, so every contract name stays on `Session`
  with no forwarding properties.
- **`LOG_DT` is at the top of `sequence.py`**, not `session.py`, so `ends.py` and `flight.py` can use it without
  importing `session.py`. The actuator helpers (`actuator`, `modules`, `joint_servos`) are there too.
- **`play.run_viewer` kept its name** (in `viewer.py`), and `load_model()` returns just the model.
- **`checks/window_recorder.py` also records the window's layout** (every widget's name, place and size), so the
  panel tidy is checked for looks as well as commands.
- **Section 16 also checks** that `setup_file.RAMPS_S` matches `Parabola`'s ramps (planned for step 5).
