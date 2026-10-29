# End effectors, setups and the class simulations: design

2026-10-28 · Wesley (with Claude Code). Status: **approved by Wesley 2026-10-29. Phase 1 is built**
(plan: `2026-10-29-end-effectors-phase1-plan.md`); phases 2–6 are not.

## Why
- We want simulations that show, for class, how CATENA moves in zero g:
  1. free float
  2. one end anchored
  3. thrusters on both ends
  4. suction cups on both ends, walking
  5. crawling along a fixed rod
  6. the skater spin
- Wesley records the clips himself. This work makes each one a setup he can open and run; recording is out of scope.
- Several of them need things on the ends of the chain that the sim can't model today. Wesley's design: **an extra
  module on each end, an end effector**, that can be an anchor, a suction cup or a thruster.
- Commands are a timetable (the sequence CSV), and they don't say how the chain was set up or what actually happened.
  This adds a **setup file** for the first and end-effector columns in the **log** for the second.

## Goals
- End-effector modules on either end, each one set to none, anchor, suction cup or thruster, from the control window
  or a setup file.
- Placing an anchored or suction end somewhere sets where the chain starts and which way it faces.
- The same sequence CSV commands the joints and the end effectors (hold, let go, nozzles).
- A setup file captures a whole run (end effectors, placement, rod, gravity, spin, sequence), so a run can be saved,
  repeated and shared.
- Example setups for simulations 1–6, each runnable from the control window.

## Non-goals
- Recording, camera work, captions, video export.
- Feedback control ("let go only once the other end holds"). Sequences stay open-loop timetables; see "Left open".
- Real end-effector hardware. The numbers here are placeholders for concepts; they stay out of `hardware/spec.toml`.

## Anti-goals (would make this a failure even if it works)
- **The chain without end effectors changes.**
  - With both ends set to none, the model and the headless log must be byte-identical to today's.
  - Every existing `verify.py` key number must stand.
- **An existing sequence file stops working.**
- **The control window becomes something Wesley has to type into.** Every setting must be reachable by clicking; text
  fields for numbers are fine.

## Constraints (from the repo)
- `make_chain.py` and `panel.py` stay stdlib-only. `tomllib` is stdlib for reading; write TOML by hand.
- `m1` stays body id 1 with the free joint.
- Actuators: joint servos first, in CSV column order.
- Log columns are only ever added at the end.
- RK4, 0.5 ms timestep; see `simulation/AGENTS.md` for the coupled settings.
- Changes to shared facts (joint count, `m1_yaw`, sequence columns) are logged in the root `LOG.md` and updated in the
  root `AGENTS.md` and `sequences/README.md`, in the same commit.

## The design

### End-effector modules (decided by Wesley)
- **Names:** `ee1` sits past m1, `ee12` past m12. Generally `ee1` and `ee{N}`.
- **Size and position:** each is one module-sized sphere (Ø139 mm, from the spec's shell size), one pitch (140 mm)
  beyond its end module's centre, along the chain axis. They are the faces m1 and m12 leave free today.
- **ee1 has a twist servo; ee12 has none (rigid on m12).** This makes the joint order the same from both ends:
  twist, bend, twist, …, bend, twist.
  - The twist servo is module 1's missing yaw. It is **`m1_yaw`**: a hinge about the chain axis between ee1 and m1,
    the same servo model and range as the other yaws (±150°).
  - It fills the `m1_yaw` column every sequence already has (always 0 today). With ee1 fitted the chain has
    **23 actuated joints**; without it, 22.
  - `m12_pitch` stays empty.
  - The lengths stay one module lopsided, as today: the first bend is 1 module in from the ee1 tip and 2 from the
    ee12 tip. Accepted.
- **Each end is set separately** to none, anchor, suction cup or thruster.
  - **none** means no module at all: that end is today's chain.
  - ee1's twist servo comes with any of the other three types.
- **Model tree:**
  - ee1 is a child body of m1 (through `m1_yaw`); ee12 is a child of m12 (no joint).
  - Add both after the rest of the chain so existing body ids don't move. The probe confirmed this: m1 = 1, m2 = 3.
  - `m1_yaw` is the first actuator when ee1 is fitted (CSV order). Suction and nozzle actuators come after all the
    joint servos.
- **Collision:** each end effector collides as one sphere, like a shell. Parent and child bodies don't collide in
  MuJoCo, so no extra excludes are needed.
- **Grip face (my call):** anchors and suction cups grip only through their tip, the outer face pointing away from the
  chain.
  - It's modelled as a flat Ø40 mm disc, a separate geom that collides only with fixed world geoms. The sphere still
    bumps into everything.
  - So a foot has to arrive tip-first to grab. A side brushing the floor or the rod doesn't latch.
  - The probe used this disc.
  - MuJoCo's `adhesion` acts on every contact of its body. The disc being its own geom with world-only collision is
    what stops a cup sticking to the chain's own modules.
- **Mass (placeholders, my call):**
  - Anchor 200 g, suction cup 200 g, thruster 250 g (nozzles, valves, gas).
  - Plus 55 g for ee1's twist servo (an AX-12A).
  - These are constants at the top of `make_chain.py`, not in the spec.
- **Look:** a colour per type, so a clip shows what each end is.

### Anchor, suction cup, thruster
| Type | What it does | Settings (defaults are placeholders) | While running |
|---|---|---|---|
| **Anchor** (a clamp) | Holds rigidly onto whatever fixed thing it touches (floor, wall plate, rod, cage). Never slips. | where it starts attached (below) | Hold / Let go; load: force (N) and torque (N·m) |
| **Suction cup** | A kind of anchor. It only grips a surface it touches, pops off above its pull limit, and holds sideways only by friction, so it can also peel off. | pull limit 20 N, cup Ø40 mm, friction 0.8, where it starts attached | Hold / Let go; pull (N) of its limit |
| **Thruster** | 5 mini nozzles: **n0** blows out of the end along the chain axis; **n1–n4** blow outward at 90° to each other, along the end module's +x, +y, −x, −y. Each pushes the opposite way to its exhaust. | max thrust per nozzle 0.1 N | each nozzle on or off; total thrust (N) |

- **Hold and Let go mean the same for anchor and suction (my call).**
  - **Hold** arms it: it grabs as soon as it touches something fixed.
  - **Let go** releases it.
  - So a foot that's told to hold before it arrives latches on arrival. This is the one bit of "wait until" the
    timetable needs.
- **Anchors and cups only grab fixed things in the world, never the chain itself (my call).**
- **Thruster physics worth showing:** every nozzle's push passes through the chain axis. So while the chain is
  straight, nozzles can move it and turn it end over end, but **can't roll it about its own length**. Rolling still
  needs the fold-and-twist loop.
- **The ee1 twist turns ee1's nozzles with it,** so it can aim them.

### Placement: where an anchored or suction end starts (decided by Wesley; details my call)
- **Settings:**
  - Attached at: a point (x, y, z in m), or a distance along the rod.
  - Facing: the surface's outward normal, one of up (floor), down (ceiling), +x, −x, +y or −y (walls).
  - Roll: degrees about that normal. It sets which way the joints fold, which matters with gravity on.
  - Leave it unset for a floating start.
- **The end effector sits on the surface with its axis along the normal, and the chain leaves the surface along that
  normal.** That sets the chain's start position and orientation.
- **The chain starts already in its sequence's first row (my call),** for example hanging from a wall, or standing on
  two feet. It doesn't swing there at full servo speed.
  - Applies only when a setup is used; plain `--seq` runs keep today's behaviour.
  - If row 0 bends the chain, the ee12 end lands wherever the chain puts it.
- **If both ends are placed, ee1 sets the pose (my call).** ee12's placement is ignored and greyed out in the control
  window; it holds whatever it lands on.
- **Surfaces:**
  - The floor already exists at z = 0.
  - A wall or ceiling placement adds a 1 m × 1 m plate there, which also collides.
  - Gravity stays along world −z.
- **Hold at the start:** an anchor or cup that starts attached starts holding.

### The rod (a scene object, my call)
- Settings: end points (m) and diameter (35 mm default, about a handrail); rod on or off.
- **Anchors clamp onto it** like onto any fixed thing.
- **That makes the rod crawl an inchworm with two clamps.** Because a clamp can't slip, it doesn't depend on guessed
  friction.
- The work is in the gait: an end has to reach the rod at the right angle to touch it.
- This replaces the "pinch hands" idea of 2026-10-28.

### Files: what to do, how it's set up, what happened (decided by Wesley; details my call)
**Sequence CSV (what to do).** The 24 joint columns stay as they are, now with `m1_yaw` live when ee1 is fitted.
Optional end-effector columns follow them. A file using them must have the header row.

| Column | Values | Used by |
|---|---|---|
| `ee1_grip`, `ee12_grip` | 1 = hold, 0 = let go | anchor, suction |
| `ee1_n0` … `ee1_n4`, `ee12_n0` … `ee12_n4` | throttle 0–1 (a solenoid valve is on/off; fractions stand for pulsing it) | thruster |

- End-effector values switch when their row starts (the start of its hold). They don't ease like joint angles do.
- A missing column keeps its start state: holding if the end started attached, otherwise off; nozzles off.
- Columns for a type that end doesn't have are ignored with one warning, as `m12_pitch` is today.
- **This is a shared format.** The robot software will read the same columns as end-effector commands. Log it in the
  root `LOG.md`, and update `sequences/README.md` and the root `AGENTS.md`.

**Setup file (how it's set up).**
- **Where:** `simulation/setups/*.toml`, the same format as `spec.toml`.
- **What it holds:**
  - the sequence file
  - move, hold and speed
  - gravity
  - spin at release (x, y, z, deg/s)
  - air drag and the net cage
  - `[ee1]` and `[ee12]`: type, placement and type settings; a missing table means none
  - `[rod]`: a missing table means no rod
- **Keys are left to the builder;** the sketch below shows the shape.
- **One line at the top says what the setup shows.** The control window displays it, so Wesley can read it before
  recording.

```toml
# Clamps on both ends crawl along a handrail.
sequence = "rod_crawl.csv"
move_s = 1.5
hold_s = 0.5
gravity_m_s2 = 0

[ee1]
type = "anchor"
attach = { rod_m = 0.3, roll_deg = 0 }

[ee12]
type = "anchor"

[rod]
from = [0, -1.2, 1.0]
to = [0, 1.2, 1.0]
diameter_mm = 35
```

**Log (what happened).**
- **New columns, appended only when an end effector is fitted,** per fitted end:
  - held (0/1)
  - load: clamp force, or cup pull (N)
  - clamp torque (N·m)
  - total nozzle thrust (N)
- **Filter the loads over 0.1 s.** A stiff clamp gives spiky raw values (see "Probe").
- **A one-line summary when a run ends** (headless, and in the viewer when the sequence finishes):
  - how far the centre of mass moved
  - IMU turn
  - peak load per end
  - how many times an end let go without being told to (a pop or a peel)
  - distance along the rod, if there is one

### Control window
- **A Setup row at the top:** pick a setup, read its line, Load (fills every field and opens the viewer), Save as.
- **An End effectors box:**
  - Per end, a Type menu. Picking a type shows that type's settings underneath (for an anchor, the placement fields),
    as Wesley asked.
  - While running: Hold and Let go, or five nozzle toggles.
- **A rod tick box with its diameter.** The rod's position comes from the setup file only (keeps the window small).
- **Rebuild + open viewer applies the fields.** It works as today: the open viewer keeps the old model until rebuilt.

### Example setups (simulations 1–6)
| Setup | End effectors | Sequence | What it should show |
|---|---|---|---|
| `1a_turn` | none | `reorient_loop.csv` at flight timing (move 1 s, hold 0.25 s), the timing the 1b numbers were measured at | turns −30.3° with no push |
| `1b_spin_0.5`, `1b_spin_1` | none | the same, let go spinning 0.5 / 1 °/s | −15°, 0°: release spin wipes it out |
| `1c_roll` | none | `roll_loop.csv`, ×2 | rolls ~63° about its length |
| `2a_arm` | ee1 anchor on a wall | new `arm_reach.csv` | reaches and holds; clamp load |
| `2b_stall_1g` / `_mars` / `_moon` / `_0g` | ee1 anchor, wall at 1.8 m, pitch axes level, starts hanging from `m1_pitch` −90° | new `stand_out.csv` (all joints to 0) | how far `m1_pitch` lifts the whole chain at each g, as an angle (static estimate: about 4° / 11° / 26° / all the way) |
| `2c_let_go` | ee1 anchor | `arm_reach.csv` plus `ee1_grip` 0 mid-swing | drifts off with the momentum the wall was absorbing |
| `3a_straight`, `_U`, `_Z`, `_L` | both thrusters | new: fold into the shape, then both n0 on | straight: nothing; U: glides; Z: spins in place; L: both |
| `3b_glide_turn` (optional) | both thrusters | U glide, reverse to stop, then a roll loop | move with nozzles, turn with folds |
| `4a_walk` | both suction cups, on the floor | new inchworm gait | walks; peak pull per foot (the cup requirement) |
| `4b_both_off` | the same | lets go of the back foot before the front one holds | floats away |
| `5a_rod_crawl` | both anchors, rod | new crawl gait | crawls along the rod |
| `6_skater` | none | `fold_sequence.csv`, spin 20 °/s about z | spin rises when folded, returns when straight |

- **The stall length is a separate measurement (my call).** The 2b clips show a lift angle, not a number of modules.
  - Experiment A's number is the stall length: the longest chain that can stand straight out level.
  - A small headless sweep script runs the 2b setup for chain lengths 2–12 at 1 g, 3/8 g, 1/6 g and 0 g, and
    reports the longest chain that stands level at each.
  - That number goes into `findings.md`, compared with the hand estimate (2 / 4 / 7 / 11 modules, from
    N(N+1)/2 · m·g·pitch ≤ 1.5 N·m with 230 g modules) and with experiment A's N_max ~ g^(−1/2).
  - Setups therefore get an optional module-count key (`build()` already takes `n`).
- **Shapes:** the U folds at m4 and m9 so that it's symmetric and the push passes through the centre of mass; the Z
  is m4 +90°, m9 −90°.
- **Thrust direction:** n0 blows outward, so it pushes its end toward the chain's middle.
- **End effectors change the numbers.** Setups with end effectors give different figures from those in
  `experiments/findings.md`, because the chain is longer and heavier.

## Build order
Each phase ends with:
- `verify.py` passing
- its setups running headless with the expected result
- its docs and `LOG.md` entries
- **one commit** (Wesley's preference: commit after substantial changes)

Push and pull requests only when Wesley says. After each phase Wesley closes and reopens Chain Sim.

- **There's no deadline yet.** The order puts the simulations that need nothing new first (1, 6), then the anchor (2),
  the experiment we'll actually fly.
- If the clips are needed by a date, move those setups forward. Cut the rod (phase 5) and 3b first.

1. **Modules and setup files.**
   - `build()` options for each end's type.
   - ee1 with `m1_yaw`, masses and colours. With both ends none, the output is byte-identical.
   - `play.py` counts joint servos instead of assuming every actuator is one: `read_sequence`, `ctrl` writes, max
     torque, straighten, module count.
   - `--setup`, and the control window's Setup row and End effectors box (types only).
   - `verify.py` section 9:
     - every type at each end compiles
     - the `m1_yaw` axis
     - zero-g momentum with end effectors fitted
     - default log byte-identical
   - Setups 1a–1c and 6.
2. **Placement, anchor, and end-effector columns.**
   - Placement and the start pose from row 0.
   - The clamp: a weld switched on and off, with "grabs on touch while holding".
   - Load readout, log columns, run summary.
   - CSV grip columns; shared-format docs.
   - Setups 2a–2c, and the stall-length sweep.
   - Measured stall length per g goes into `findings.md`.
3. **Suction cup.**
   - MuJoCo's `adhesion` actuator, limited to world geoms.
   - Pull readout; pops and peels counted.
   - Setups 4a, 4b.
   - Peak foot pull goes into `findings.md`.
4. **Thruster.**
   - Five nozzle sites per end; nozzle columns; nozzles light up when firing.
   - Setups 3a (and 3b).
   - Checks: one n0 on a straight chain gives a = F/m; the U glides with little spin; the Z spins with little drift.
5. **Rod.**
   - The rod object; clamping onto it.
   - The crawl gait, iterated until it advances.
   - Setup 5a.
6. **Wrap-up.**
   - `experiments/README.md` ("not in the sim yet").
   - A `findings.md` section with the measured numbers.
   - A final `verify.py`.

## Probe (2026-10-28, MuJoCo 3.16, RK4 0.5 ms; throwaway script, not in the repo)
- **ee1 on `m1_yaw` and ee12 rigid:**
  - The chain compiles with 23 actuators, m1 = body 1, existing ids unchanged.
- **Clamp (a weld to the world):**
  - With the default weld softness, ee1 moved **36 mm** while the chain folded.
  - With `solref 0.002` it moved **0.28 mm**; with 0.001, 0.035 mm.
  - Raw peak loads rise with stiffness (3.0 → 9.6 → 13.3 N; 3.3 → 20.8 → 29.5 N·m), so they need filtering and a
    sanity check before they go on a slide.
  - Switching it off mid-run works (`eq_active`).
- **Suction (`adhesion`, 20 N):**
  - Held a straight pull of 15 N; let go at 19 N.
  - Pushed sideways at the sphere centre, it **peeled**, from 5 N up: the push tips the module about the Ø40 mm cup.
    That's real suction-cup behaviour, so cup size matters.
- **Nozzle (site actuator, 0.1 N on 0.6 kg):** moved exactly ½·a·t² (0.0833 m in 1 s).

## Left to the builder
- Setup-file key names, log column names, run-summary wording.
- The control window's layout.
- Colours, plates, nozzle visuals.
- Weld and contact tuning (start from the probe), and load filtering.
- Gait keyframes for 2a, 4a, 5a.

## Left open, for later
- **Feedback in sequences** ("let go only once the other end holds") beyond grab-on-touch. If the crawl or the walk
  needs it, add one "wait until held" option in the setup or a small Python controller, rather than growing the CSV.
- **Real end-effector numbers.** When one is designed, its mass and limits move to `hardware/spec.toml`.

## Calls made by Claude (check these)
1. Each end can be none, so today's chain stays the default.
2. Placeholder masses: 200 / 200 / 250 g, plus 55 g for the ee1 servo.
3. Hold means "grab on touch"; anchors and cups never grab the chain itself.
4. The nozzle layout: n0 axial out of the end; n1–n4 along +x, +y, −x, −y.
5. Placement by point, facing and roll; ee1 wins when both ends are placed.
6. A setup starts the chain in its sequence's first row.
7. The rod crawl uses rigid clamps on a rod, not pinch hands.
8. Setups live in `simulation/setups/`; the design doc lives in `docs/design/`.
9. Grip and nozzle values switch at the start of a row.
10. Anchors and cups grip only through a flat Ø40 mm tip disc.
11. The 2b clips show a lift angle; the stall length comes from a separate sweep over chain length.
