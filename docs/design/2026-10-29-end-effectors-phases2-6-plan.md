# End effectors, phases 2–6: implementation plan

**Goal:** all six class simulations run from the control window (Load, then Play). Wesley asked on 2026-10-29: "by
the end I want to see 6 different sequences that I can see how they work".

**Spec:** `2026-10-28-end-effectors.md`. Phase 1 (`2026-10-29-end-effectors-phase1-plan.md`) is built.

**Execution:** inline. One commit per phase after `verify.py` passes. Push and PR only when Wesley says. The
ledger is `.superpowers/sdd/2026-10-29-end-effectors-phases2-6-plan/progress.md`.

## Global constraints (from the spec, unchanged)
- Both ends none → `build()` XML and headless logs byte-identical to `fbe880f`. Every key number in sections 1–9
  stands.
- `make_chain.py`, `setup_file.py` and `panel.py` stay stdlib-only. Log columns are only appended, and only when an
  end is fitted.
- m1 stays body 1; new bodies (hold mocaps, cups) go after the chain. Joint servos are the first actuators.
- End-effector actuators override the servo defaults: `biastype="none" dyntype="none"`, `forcelimited="false"`.
  The probe showed `adhesion` inherits ±1.6 N otherwise.

## Decisions (mine, from the spec's "left to the builder")
- **Setup schema per end:** `[ee1]` holds `type` and `attach`, plus the settings for that type:
  - attach: `{ at = [x, y, z], facing = "up", roll_deg = 0 }`, or `{ rod_m = s, facing = "up", roll_deg = 0 }`,
    or missing (floating)
  - suction: `pull_N = 20`, `cup_mm = 40`, `friction = 0.8`
  - thruster: `thrust_N = 0.1`
  - `[rod]` holds `from`, `to`, `diameter_mm = 35`.
  - In the setup dict, `ee1`/`ee_n` become dicts with keys `type, attach, pull_N, cup_mm, friction, thrust_N`.
  - `build()` accepts a type string (phase 1 callers) or such a dict.
- **Facing** is the surface's outward normal: up, down, +x, −x, +y, −y. The chain leaves the surface along it.
  - The pitch axis starts level: world x for up, down, ±y; world y for ±x. `roll_deg` turns it about the normal.
  - On a rod, facing is the side of the rod the chain is on, and must be perpendicular to the rod.
- **Plates:** a 1 m × 1 m × 10 mm world box at a point placement, unless it's on the floor (facing up, z = 0).
  `statistic center` moves to the placed chain.
- **Start pose:** `Session.use_setup(setup)`.
  - Joints, `ctrl` and servo `act` are set to the sequence's row 0.
  - If one end is placed, the root is moved so that end's frame sits on the surface. ee1 wins if both are placed.
  - That end starts holding.
  - `reset()` (Reset pose) goes back to this.
  - Plain `--seq` runs and `verify.py`'s `Session(m, d)` are unchanged.
- **Clamp:** each anchor end has a mocap body `ee{k}_hold` and a weld `ee{k}_clamp` (relpose identity,
  `solref 0.002 1`, inactive).
  - Clamping moves the hold body to the end's current pose and switches the weld on. Let go switches it off.
  - Holding = armed. An armed, unclamped anchor clamps when its tip disc (geom `ee{k}_tip`, Ø40 mm, 4 mm thick,
    collides with nothing) is within `CAPTURE = 15 mm` of a world geom (`mj_geomDistance`). The sphere itself still
    collides as before.
- **Suction:**
  - A child body `ee{k}_cup` carries the cup geom. It has `contype 2, conaffinity 0`, its own margin and gap (10 mm,
    so the cup pulls in from 1 cm), and friction 0.8.
  - World geoms get `conaffinity 3` when a suction end is fitted, so cups touch only world geoms, never the chain.
  - `adhesion` (name `ee{k}_suck`, `body="ee{k}_cup"`) has gain = pull limit.
  - Held = on and the cup in contact. A pop is held → not held while on. Pull = adhesion − normal contact force.
- **Thruster:**
  - Sites `ee{k}_n0` … `ee{k}_n4`. n0 is at the tip, exhausting outward along the axis. n1–n4 are on the equator,
    along +x, +y, −x, −y of the end's frame.
  - `general` actuators with the same names: `gear 0 0 -1 0 0 0`, gain = thrust_N, ctrl 0–1.
  - A site turns bright while it fires.
- **CSV columns** `ee{k}_grip` and `ee{k}_n0`…`n4`, each read by `read_ee()`. Row k's value applies from the start of
  row k's hold, until row k+1's hold begins. `read_sequence` skips them silently.
- **Log columns** per fitted end, appended: `ee{k}_held, ee{k}_load_N, ee{k}_torque_Nm, ee{k}_thrust_N`. Loads are
  filtered (exponential, τ = 0.1 s).
  - Summary per end: peak load and pops. With a rod: COM distance along the rod.
- **Panel ↔ viewer (fixes the review's I1):**
  - The panel lays its fields over the loaded setup and writes the result to `panel_setup.toml` (git-ignored).
  - It launches `play.py --setup panel_setup.toml --panel PORT`, so both paths share one mapping. `chain.xml` is no
    longer written by the panel.
  - In panel mode the setup's sequence doesn't autoplay, and its spin waits for Play.
  - New commands: grip {end, on}, nozzle {end, n, on}.

## Phase 2: placement, anchor, grip columns
- setup_file schema, plus the review minors:
  - relative sequence paths inside the repo
  - `isfile`
  - a strict `toml()`
  - file names on syntax errors
  - a clear message for a non-table `eeK`
- `build()`: tip disc, hold mocap, weld, plates, rod geom.
- `play.py`: ends, start pose, clamp/grab/let go, grip columns, log columns, summary, readouts, the grip command.
- `verify.py` section 10, anchor checks:
  - placement puts the tip on the surface
  - the clamp holds within 1 mm while the chain folds
  - it grabs on touch
  - letting go keeps the momentum constant
  - none still gives an identical log
- Sequences: `arm_reach.csv`, `arm_let_go.csv`, `stand_out.csv`.
- Setups: `2a_arm`, `2b_stall_1g` / `_mars` / `_moon` / `_0g`, `2c_let_go`.
- `stall_sweep.py`; its result goes into `findings.md`.
- The panel: merged setup, Hold / Let go, readouts.

## Phase 3: suction
- Cup body and adhesion. The check: holds a straight 15 N pull, pops below 21 N.
- `inchworm_walk.csv`, `walk_let_go.csv`; setups `4a_walk`, `4b_both_off`. Peak pull goes into `findings.md`.

## Phase 4: thruster
- Nozzles and lights.
- Checks:
  - one n0 on a straight chain gives a = F/m
  - straight, both on: no drift
  - U: glides, little spin
  - Z: spins, little drift
- `thrust_straight/U/Z/L.csv`; setups `3a_straight`, `_U`, `_Z`, `_L`.

## Phase 5: rod crawl
- Iterate `rod_crawl.csv` until it advances along the rod; setup `5a_rod_crawl`.

## Control window (after phase 5)
- Each end: the Type menu shows that type's settings.
  - Anchor and suction: placement (floating / point / rod, x y z, facing, roll). ee12's placement greys out when
    ee1 is placed.
  - Suction: pull, cup, friction. Thruster: thrust.
  - Running: Hold / Let go, or five nozzle toggles.
- A Rod tick box with its diameter.
- A scripted test of the real window.

## Phase 6: wrap-up
- `experiments/README.md`, a `findings.md` section, the docs, a final `verify.py`, one fresh review.
