# Log: simulation

Newest first. The format is in the root `AGENTS.md`.

## 2026-11-03 · Wesley (Claude Code)
- **Refactor, phase 2: the run in five files.** `play.py` (1,066 lines) is now `sequence.py` (the CSV format, `LOG_DT`),
  `ends.py` (`End` with its behaviour: hold, latch, capture, cups, nozzles, loads), `flight.py` (`Parabola` and
  `FlightBox`), `session.py` (`Session`, `Logger`, `load_model`) and `play.py` (265 lines: the command line and the
  viewer). `verify.py` and `stall_sweep.py` import from the new files and use `load_model`.
  - Not as planned: `Flight` as a separate object would have needed most of `Session`'s insides (spin, inertia, play,
    the ends, the shells, the sequence state), so by the plan's stop rule its methods moved as a block, `FlightBox`,
    which `Session` inherits. Every contract name stays on `Session` with no forwarding.
- **Checked:** `checks/same_as_before.py compare`: 48 of 48 the same (verify ALL CHECKS PASSED, fold and 21 setup logs
  byte-identical, XML, window, stdlib). `stall_sweep.py`: 3 / 4 / 6 / 12+ modules at Earth / Mars /
  Moon / zero g, as in `../experiments/findings.md`.

## 2026-11-03 · Wesley (Claude Code)
- **Refactor, phases 0 and 1** (`../docs/design/2026-11-03-refactor-plan.md`; Wesley: "your plan looks good. Carry on").
  - `checks/same_as_before.py` (with `checks/window_recorder.py`): save a snapshot before a change, compare after.
  - The model in three files: `make_chain.py` keeps `PRESETS`, the chain's numbers and `build()`; the end effectors'
    XML and numbers moved to `make_ends.py`, the flight box's to `make_box.py`; `spec.py` reads the spec once. `build()`
    is 156 lines instead of 232, and its signature is unchanged. Imports updated in `play.py`, `setup_file.py`,
    `panel.py`, `verify.py`, `stall_sweep.py`.
- **Checked:** `checks/same_as_before.py compare`: 48 of 48 the same as the baseline from `292d939`: verify.py's
  printout (ALL CHECKS PASSED), the fold log and all 21 setups' logs and printouts byte-identical, the XML of 47 models,
  every button's commands, and the stdlib-only check.

## 2026-11-03 · Wesley (Claude Code)
- **Stale docs fixed** (refactor plan, section 6): `setups/1d_flight_turn.toml` (gone) replaced by `1a_turn` in
  `AGENTS.md`, `README.md` and `play.py`'s docstring; README: four presets, `stall_sweep.py` builds its own setups,
  `pole_climb_design.py` writes only the sequence, kv is open issue 10, the opening mentions the flight box, the GUI
  tests of 2026-11-03; AGENTS.md: what `verify.py` imports from `play.py`.
- **Checked:** `verify.py` ALL CHECKS PASSED. Docs and a docstring only.

## 2026-11-03 · Wesley (Claude Code)
- **Demo button 1B** (Wesley: "1B alongside 1A free float on the demo... I want a sequence"): **1A Free float (held)**
  and **1B Free float (from the net)** side by side, then 2–6; one click flies 1b_from_net with its turn sequence. With
  no strings the readout says "none (lying on the net)" instead of "holding (0)".
- **Checked:** the real control window and viewer, driven by script: clicking 1B counted down, flew the pull-up with
  the chain on the net, and played reorient_loop.csv from T+1.5 s to 100% at T+21.7 s, in one viewer process; Start
  the parabola by hand on 1b ran the countdown and the pull-up the same way (an earlier run showed a stray viewer state
  once and didn't repeat). `verify.py` not re-run for this (window and readout text only).

## 2026-11-03 · Wesley (Claude Code)
- **By hand, say what's next** (Wesley: "when I click start the parabola... I am not seeing anything"; it was working,
  but for ~30 s nothing moves in the cabin view and the sequence waits for Play). The viewer's big text now counts down
  to zero g ("1.8 g pull-up - zero g in 12") after the T-5 countdown, then says "Let go when ready" and "Play: the
  sequence"; a **by hand** readout line says the same.
- **1b starts on the net** (Wesley: a free float "from the net, not in the middle held by the cables"): `[flight]
  on_net` lays the chain on the net with no strings, and the window has an **or starts on the net** tick. The
  free-float setups move up a letter: `1b_from_net`, `1c_spin_0.5` / `1c_spin_1`, `1d_roll`, `1e_turn_across`.
  - The net now has the floor's rolling resistance: lying on its side, the chain rolled ~60 mm onto its heavy side
    (each module's servo block is off its axis). 1b starts in that resting roll (`roll_deg = -90`, pitch axes across).
  - `liftoff` records when and how fast it leaves the net; touches count only once the chain has been clear.
  - Finding: the leftover g pulls it off the net at T+2.4 s at 210 mm/s, into the lid at T+6 s; with no jitter it
    still drifts into an end wall by T+9 s (the net is 0.65 m below the box's turning centre). Held in the middle by
    strings it first touches at T+20.9 s.
- **Checked:** `verify.py` ALL CHECKS PASSED. All 21 setups flown headless (landings now sink at most 90 mm). The window
  by script: 1b loads with the tick on and hands the viewer exactly its setup. The new prompts read right in a headless
  by-hand flight (countdown to zero g, Let go, Play). Not seen in the live viewer.

## 2026-11-03 · Wesley (Claude Code)
- **Every setup flies** (Wesley: "modify all the current experiments... add the flight sequence and the box into them";
  ~20 s of experiment after a few seconds to let go, then the fall to the net). All 20 setups are flights on
  `feetech85`, held by strings (or their anchor) through 1.8 g, let go at T+1 s, the sequence from T+1.5 s. Per demo:
  anchors on the box's end wall (2a, 2c with strings; 2b without, hanging); 2b as 0 g, Moon, Mars and 1 g parabolas
  (`[flight] level_g`); thrusters at 0.025 N (the same push per kg); the walk on a deck over the net; the pole Ø61 mm
  from the box's floor to its lid; a Ø25 mm rail; 1d is now the turn with the pitch axes across the cabin
  (`[flight] roll_deg = 90`). The old 1d_flight_turn and the seventh demo button are gone; the six buttons fly.
- **Manual mode** (Wesley: buttons to "simulate the zero g flight" and "release the cable"): Start the parabola (no
  sequence, strings holding), Let go of the strings (`release`), and Play mid-flight plays the sequence at once (no
  reset, no countdown, which would hold the physics). A setup's spin is added as the strings let go.
- **Camera toggle** (Wesley: the camera turning with the plane or not): **Camera turns with the plane**, the `camera`
  command; unticked, a free camera fixed in space shows the box pitch.
- **The box's look** (Wesley: "slightly more detail... aluminium extrusion"; representation only): extrusion on every
  edge and up the long sides, the net's frame and 50 mm mesh, a door on the aisle side, feet. The net is 300 mm above
  the floor (the buffer for a landing) and the strings hold the chain in the middle of the room above it.
- **The cabin is a real body on a hinge** (`cabin_pitch`, 10 t), driven each step, not a mocap body: MuJoCo gives mocap
  bodies no velocity, so the walk slid 0.5 m on the deck as the box pitched. Clamped anchors' holds ride on it.
- **Suction cups are square pads.** A disc on a box surface (the deck) rocked under the tipping moment and the walk
  failed even in perfect zero g; on a plane discs and pads match (14 cm walk 1.104 vs 1.108 m; 40 and 60 mm cups still
  peel).
- verify.py section 13 and `stall_sweep.py` build their perfect-zero-g 14 cm setups in code; `pole_climb_design.py` writes
  only the sequence. The strings are tied on whenever the chain is at its start.
- **Checked:** `verify.py` ALL CHECKS PASSED, 15 sections (15 adds the deck walk, 671 mm with no pops, and a Mars
  parabola; 13 still climbs 394 mm). `fold_sequence.csv`'s headless log byte-identical to this morning's. All 20 setups
  flown headless (results in `../experiments/findings.md`). 1d without jitter turns −118.1°, a free chain let go at the
  same spin −118.2°. The control window by script: the six buttons hand the viewer their setups and fly; Start, Let go,
  Play mid-flight and the camera tick send what they should. Offscreen renders of the box looked right. Not run in the
  live viewer.

## 2026-11-03 · Wesley (Claude Code)
- **The plane's pitch is modelled** (Wesley: "model the flight angle etc too"). The box, net, string ends and a cabin
  camera are one mocap body, `cabin`, turned about the across axis with the plane's pitch (nose up ~49°, over the top at
  up to 5.2°/s, nose down), and gravity is turned with it; the world moves with the box but doesn't turn. The viewer
  starts on the cabin camera. `turned` is in the box's frame; `inertia()` sums the chain only (the cabin's box geoms
  had taken a density's mass and swamped it: the cabin's geoms are massless now).
- **A flight setup flies on Play** (Wesley: the experiment "written into the sequence, pre-recorded"): its `[flight]`
  holds the release and start times, so Play, Restart, a demo button or a headless run fly the whole parabola. The
  separate Fly button and `--fly` are gone. New demo button **✈ On the flight** (`1d_flight_turn`).
- Box 2.0 × 1.3 × 1.9 m (spec and `1d_flight_turn`).
- **Finding:** let go from the turning box, the chain keeps the plane's pitch rate (−2.7°/s about the across axis),
  and folding turns that into roll about its length: the 30° turn comes out at −5°. A free chain at 0.5°/s about that
  axis already turns −20.9° instead of −30.3° (same at half the timestep and on the 14 cm chain). In
  `../experiments/findings.md`, "The flight box".
- **Checked:** `verify.py` ALL CHECKS PASSED, 15 sections (15 now checks the chain stays within 1.1 mm in the box as the
  plane pitches up, leaves turning with the plane to 0.3°/s, lands above the box floor, and the 0.5°/s across-axis
  turn of −20.9°). Logs byte-identical for `fold_sequence.csv` and the 19 older setups. The control window by script
  (viewer stubbed). Not run in the live viewer.

## 2026-11-03 · Wesley (Claude Code)
- **The flight box and a whole parabola** (Wesley: an acrylic box with a net beneath, the chain held still in the middle
  by "a few string tethers... releasing upon we hit zero g"):
  - `make_chain.build(flight=)`: the box (stiff walls and lid) with its floor at z = 0, the net as a soft block from the
    floor up to its height, and four strings per held module (tendons with a length limit) to the top and bottom edges
    of the box's long sides, fanned along the chain. Strings straight up and down were tried first: the leftover g
    swung the chain ~10 mm sideways and it left at 17 mm/s.
  - The net's contact: impedance 0.05 with a direct spring and damping ratio 0.3. A timeconst solref settled a resting
    shell 0.4 mm deep instead of 20 mm, because MuJoCo's soft contact cancels most of gravity; MuJoCo also divides a
    direct stiffness by the impedance. Landings now sink as deep as on a 20 mm-sag net; at rest it's ~3 mm deep.
  - `play.Parabola` (the ITF's 1 g / 1.8 g / 22 s zero g / 1.8 g, with jitter waves timed from T = 0 so every take is
    the same) and `Session.fly`: hold, let go at T+release_s, start the sequence at T+start_s, track the gap to the box,
    the first touch, the landing. Readouts, a big g / T banner in the viewer, 5 log columns, `play.py --fly`, the
    `fly` command, and the control window's **Flight** box with **Fly a parabola**.
  - Setups gain a `[flight]` table; the old `cage` key, `--cage` and the panel's Net cage tick box are gone (the box
    replaces the cage). New setup `1d_flight_turn`.
- **Preset `feetech85`:** the flight build (Wesley: 8.5 cm modules, 0.5 mm apart), the AX-12A module scaled by
  85.5/140 (Ø84.9 mm shells), STS3032, masses scaled (61 g base, 7 g arm), timestep 0.2 ms.
- Results in `../experiments/findings.md`, "The flight box"; the cabin limits in `../experiments/flight_space.md`.
- **Checked:** `verify.py` ALL CHECKS PASSED, now 15 sections (~100 s): section 7's cage check is now the box, and 15
  flies `1d_flight_turn` (held within 0.36 mm at 1.8 g, a clean release, −30.1° with the jitter, lands at 3.2 m/s without
  reaching the floor). `feetech85` passes the momentum (0.0004 mm) and floor-drop checks. With no box, `build()` is
  byte-identical to before for all presets, and the logs of `fold_sequence.csv` and all 19 older setups are
  byte-identical. The control window by script (viewer stubbed): Load 1d hands the viewer exactly its setup, Fly sends
  reset, speed, playback, countdown, fly; Fly without a box and a bad string list are refused. Offscreen renders of
  the box at 1.8 g, in zero g and after landing looked right. Not run in the live viewer.

## 2026-11-03 · Wesley (Claude Code)
- **Cleanup after the demo week** (Wesley: "make sure that someone who is trying to use the engine could use it
  easily"):
  - **Nothing had been taken out of the control window.** Every control from before the demos is still there. The only
    UI removed was the separate, simplified Demos window, dropped on 2026-10-29 at Wesley's request ("I want the old
    user interface and control").
  - **Setup files show only what's special:** `save` writes sequence, move_s, hold_s and whatever differs from
    `DEFAULTS`, and all 19 setups were rewritten that way. `5a_pole_climb` is now 4 lines plus its `[rod]` table, which
    makes it plain that the pole is a setup setting, not part of the CSV.
  - **Consistent names,** `{demo}{version}_{what}`: `5_pole_climb` → `5a_pole_climb`, `5a_rod_crawl` → `5b_rod_crawl`,
    `6_skater` → `6a_skater`, `4b_both_off` → `4b_walk_let_go`.
  - **Control window:** under Setup, a line on what's in it (`setup_file.summary`: world, chain, end effectors, how it
    starts); under Fold sequence, the CSV's first line and the setups that use it (`setup_file.users`). It opens on
    `fold_sequence.csv` again; it had been picking `arm_let_go.csv`, first alphabetically, which needs an anchor. The
    demo list moved to `setup_file.DEMOS`.
  - `play.py --setup` with a bad file prints one line instead of a traceback, and a headless setup run prints what's
    in the setup. `Session.place` finds the setup's ends by name (`end_keys`); same behaviour.
  - `README.md`: "How a run is put together" and "Make your own run".
- **Checked:** `verify.py` ALL CHECKS PASSED, 14 sections (new checks: the minimal save, every demo button has its
  setup, every CSV has a first line, `pole_climb.csv` → `5a_pole_climb`). Logs byte-identical to before for
  `fold_sequence.csv` and all 19 setups headless (the renamed ones against their old names). `pole_climb_design.py`
  writes the same CSV (only the setup name in a comment changed). The control window by script, with the viewer
  stubbed out: each of the six buttons hands the viewer exactly its setup and plays on connect, and the new lines read
  right. Not looked at by eye.

## 2026-10-29 · Wesley (Claude Code)
- **Countdown before a take** (Wesley: "a 5 seconds delay before the experiment start... T-5 4 3 2 1"; then "can this
  value be something that is adjusted on the chain sim panel"). Play and Restart send `countdown {s}` (the panel's
  new **Countdown s**, 5 to start with, 0 for none). The viewer draws T-n big at the top and holds the physics and the
  take's spin and play (`play.Commands`) until T-0; Pause, Stop, Reset or quit during it call the take off.
- **The viewer window reopens where it was** (size and place): GLFW's `COCOA_FRAME_NAME` hint, so macOS saves the frame.
- **The viewer's UI size can't follow it between displays**: MuJoCo picks it once, as the window opens (150% on the
  HP monitors, judging density from the Retina; 200% = normal on the Retina), and its Python API can't change it.
  Documented the workaround (Option → Font, or reopen the viewer there) in `README.md`.
- The sequence readout shows its length to 0.1 s, so `reorient_loop` reads "20.2 s: longer than a 20 s parabola"
  instead of "20 s: longer than a 20 s parabola".
- **Checked:** `verify.py` ALL CHECKS PASSED, 14 sections (new 14: the spin and play wait out the countdown, Stop calls
  it off, Play again starts it afresh). Live viewer with a stand-in panel: sim time held at 2.2 s through "starts in
  5 ... 1 s", the take started at T-0; Wesley saw the big T-25 of a 40 s test. A test viewer reopened at a saved
  700×500 frame. Panel: Play sends `countdown 3` before play for 3, none for 0, 5 for an empty box; the panel's width
  is unchanged (1146 px). Not seen: the new panel line on screen (screenshot declined).

## 2026-10-29 · Wesley (Claude Code)
- **New demo 5, the pole climb** (Wesley: "a snake swirling through a much bigger rod... wrap and inch along"). Setup
  `5_pole_climb` and `pole_climb.csv`: the bare chain wrapped around a Ø100 mm pole climbs it, no cups or clamps. Button 5
  plays it; the clamp crawl (`5a_rod_crawl`) is in the Setup list.
  - The keyframes were designed on the pole's surface (module centres on cylinders round it, joint angles from
    those points), because giving each zone its own coil put the front 39–265 mm off the pole.
  - `[rod] wrap_m` starts a setup wrapped: the sequence's first row is a coil, and `Session.wrap` puts its axis on the
    rod's.
  - Measured: 394 mm up in 19.3 s; the squeeze holds the servos at their 1.6 N·m limit (a real AX-12A would overheat).
- **Checked:** `verify.py` ALL CHECKS PASSED, 13 sections (new: the wrapped start touches all round, the climb gets up
  the pole); button 5 writes the same setup the viewer needs and that file climbs the same headless. The live window
  couldn't be tried: the Mac's display was idle and the viewer didn't open.

## 2026-10-29 · Wesley (Claude Code)
- **Six demo buttons at the top of the control window** (Wesley: "a simple button for me to play each sequence"):
  1 Free float (`1a_turn`), 2 Anchored (`2a_arm`), 3 Thrusters (`3a_U`), 4 Suction walk (`4a_walk`), 5 Rod crawl,
  6 Skater. One click loads the setup, opens the viewer and plays; the other versions stay in the Setup list. A
  separate simplified "Demos" window was tried and dropped: Wesley wanted the existing window.
- The panel's viewer link (socket, starting the viewer, commands, readouts) moved to `viewer_link.py`. End-effector
  readouts read "thruster, firing/off, … N" and "suction cup, …" instead of repeating the type.
- **Checked:** `verify.py` ALL CHECKS PASSED; the real window by scripted buttons: one click on 4 and on 3 opens and
  plays each, Play again restarts from the start, and the retake test still passes.

## 2026-10-29 · Wesley (Claude Code)
- **Fixes from a fresh review of phases 2–6:**
  - A second take (Play again, or Restart) now starts from the start pose. Before, it carried on from where the last
    take ended: 5a crawled off the end of the rod, 4a tumbled.
  - Pause, Stop and Straighten turn the nozzles off (a stopped burst kept accelerating the chain); Resume re-applies
    the current row.
  - With both ends attached, the second end now holds whatever it lands on, as the design and the panel say.
  - Left for later (minor): the panel's end names go stale if Modules changes without Rebuild; nozzle tick boxes
    don't follow the sequence; `rod_m` isn't range-checked; anchors latch tilted up to ~60°; the viewer's default
    camera crops some clips; a thruster's readout says "thruster, thruster"; `turned` means little when anchored.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; the real control window by scripted buttons: Load 5a, Play
  ×8 to the end, Play again and Restart both start afresh (identical drift at the same point, 308 mm).

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 6: wrap-up.** `README.md` covers the end effectors, every setup (grouped as the six class
  simulations), Playback x and the new log columns. `../experiments/findings.md` has "The class simulations" (each
  clip's measured result, the stall length, the suction-cup tipping requirement, the side-nozzle roll), and
  `../experiments/README.md` moves the held end and the anchored arm out of "Not in the sim yet".
- **Checked:** every one of the 18 setups headless gives the result its description says; filmstrips of 11 of them
  rendered offscreen and looked at (in `renders/clips/`, not in git).

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 5: the rod crawl.** `rod_crawl.csv` and setup `5a_rod_crawl`: clamps on both ends crawl along
  a Ø35 mm rod with the same arch gait as the walk (landing exactly on the rod; a clamp grabs from 15 mm and pulls in).
  It moves 1.115 m along the rod in 19 s (two steps), one clamp always holding, peak clamp load 3.7 N.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; 5a headless.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 4: thrusters.** Five nozzle sites per thruster end (n0 out along the axis, n1–n4 along the
  end's +x, +y, −x, −y), each a site actuator pushing against its exhaust, lit while firing. Sequences
  `thrust_straight/U/Z/L.csv` and setups 3a ×4: a half-throttle burst of both n0, then coasting.
  - Results: straight doesn't move; the U glides 0.93 m and turns 1°; the Z spins at 8.2°/s and its centre of mass
    moves 0.16 mm; the L moves 0.66 m and turns 43°.
  - Finding: every nozzle's push goes through the chain's axis, but each module's centre of mass sits ~4.8 mm off that
    axis (the servo block isn't centred in the shell). So n2/n4 turn it with no roll, while n1/n3 also roll it,
    ~12.8°/s after 2 s of 0.1 N, because the chain is so easy to roll about its length.
  - 3b (glide, then turn) skipped: optional in the design, and 3a covers the thrusters.
- **Checked:** `verify.py` ALL CHECKS PASSED, 12 sections; the four 3a setups headless.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 3: suction cups.** A cup body at the end's tip with an adhesion actuator (the pull limit),
  touching only fixed things through explicit contact pairs; held = sucking and on a surface; pull = suction minus the
  surface's push; pops counted. Setups 4a (inchworm walk, `inchworm_walk.csv`) and 4b (lets go too early,
  `walk_let_go.csv`).
  - Finding: the default 40 mm, 20 N cup peels under the stepping moment and the chain pivots off; 80 mm, 60 N walks
    (1.10 m in two steps, 19 s). The pull it feels is only ~1 N: what sizes the cup is the tipping moment, about
    pull × radius ≈ 2.4 N·m.
  - MuJoCo 3.16 doesn't let `gap` switch off a contact's push inside its margin, so cups have a 1 mm margin and gaits
    press each landing foot ~15 mm into the floor (see Gotchas in `AGENTS.md`).
- **Checked:** `verify.py` ALL CHECKS PASSED, 11 sections; 4a walks two steps with no pops, 4b floats away.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors, phase 2: placement and the anchor** (plan `../docs/design/2026-10-29-end-effectors-phases2-6-plan.md`):
  - Setups can start an end attached (`attach`: a point and facing, or along the rod), and start the chain in its
    sequence's first row. `[rod]` adds a fixed rod.
  - The anchor clamps (a weld to a mocap body), grabs on touch when told to hold (its tip within 15 mm, then pulls in
    softly over 0.2 s), and lets go; load and torque are smoothed and logged (4 columns per fitted end).
  - Sequences can carry `ee{k}_grip` columns. New: `arm_reach.csv`, `arm_let_go.csv`, `stand_out.csv`; setups 2a,
    2b ×4, 2c. `stall_sweep.py`: the longest chain that stands out level is 3 modules on Earth, 4 on Mars, 6 on the
    Moon, all 12 in zero g (hand estimate 2 / 4 / 7).
  - Wesley asked for a speed multiplier for watching: **Playback x** (panel), `--playback`, setup key `playback`. It
    runs more physics steps per frame, so the physics doesn't change; the readout says if the computer can't keep up.
  - The control window lays its fields over the loaded setup and starts the viewer from `panel_setup.toml` (the
    review's I1), and has per-type end-effector settings, Hold / Let go, nozzle toggles and a rod tick box. Two
    columns now.
  - The phase 1 review's minors in `setup_file.py` are fixed (relative paths, strict `toml()`, `isfile`, file names
    in errors).
- **Checked:** `verify.py` ALL CHECKS PASSED, 10 sections; with both ends none, `build()` and the headless logs are
  byte-identical to `fbe880f`; the 1a setup log equals the plain-flags run; every new setup headless; the real
  control window by scripted buttons (Load 2a, Play ×4, Let go, Hold, Save as keeps the placement). Not looked at by eye.

## 2026-10-29 · Wesley (Claude Code)
- **End effectors and setups, phase 1** (design `../docs/design/2026-10-28-end-effectors.md`, plan next to it):
  - `build(..., ee1, ee_n)`: an optional module past m1, on the `m1_yaw` twist servo, and one past module N, rigid.
    Each is none, anchor, suction or thruster; so far they differ only in mass (placeholders in `make_chain.py`) and
    colour.
  - `play.py` counts joint servos and modules instead of assuming every actuator is a joint (`modules`,
    `joint_servos`, `Session.nj`). Headless runs end with a summary line.
  - Setup files: `setup_file.py`, `play.py --setup`, and five example setups in `setups/` (1a, 1b ×2, 1c, 6). The
    control window has a Setup box (Load, Save as), an End effectors box, and "spin when Play starts".
  - 1a/1b use flight timing (move 1 s, hold 0.25 s), not ×4, because the release-spin numbers in `findings.md` were
    measured at that timing.
  - `AGENTS.md` was over 250 lines: "Physics choices" moved to the end of `README.md`.
  - From a fresh review: setups refuse `move_s <= 0`, `hold_s < 0` and `speed <= 0` (they hung a headless run or
    crashed the viewer), and verify's random motion moves only joint servos. The rest is open issue 9 in `AGENTS.md`.
- **Checked:**
  - `verify.py`: ALL CHECKS PASSED, now 9 sections; sections 1–8 print exactly what they did before.
  - Both ends none: `build()` output and the headless logs (fold sequence; flight-like with spin, air, cage) are
    byte-identical to `fbe880f`.
  - Setups headless: 1a turned −30.3°, 1b −15.2° / 0.0°, 1c IMU turn 63.4°, 6 back to +20.0°/s, matching `findings.md`.
  - The real control window, with scripted button presses: Load 6_skater → Play (spin +20.0°/s), ee1 thruster →
    Rebuild (`chain.xml` has `ee1` and `m1_yaw`), Save as → Load gave the same fields. Not yet looked at by eye.

## 2026-10-22 · Wesley (Claude Code)
- **Fixes from the outside review:**
  - Spin now changes one axis at a time; it used to reset the other two. Stop spin zeroes all three.
  - The servo force limit now includes the gear friction, so the net output at stall is the rated stall torque. It was
    7% low. The key numbers in `verify.py` didn't move.
  - Docs: `kv` brakes during all unsaturated motion, not only near the target. Rotor momentum is about 3% for the
    reaction-wheel finding.
- **New flight options:**
  - **IMU output:** sensors `imu_quat` and `imu_gyro` on the module in `[imu]`, 7 new log columns, and an `IMU turn`
    readout.
  - **Spin about x, y or z:** the panel's axis choice, or `--spin X Y Z`, for leftover spin at release.
  - **Air drag and the net cage:** `--air` and `--cage`, or the panel tick boxes. Their numbers are in `[flight]`.
  - **`--speed`:** the sequence readout shows the sequence's length and flags anything longer than one parabola.
  - **`module_g`** in the spec, for weighed modules.
- **Checked:**
  - `verify.py` ALL CHECKS PASSED (now 8 sections).
  - Real launch through the real buttons: ticked Air drag and Net cage, Rebuild, Play, Spin about the across axis. 3 of
    4 runs completed every step; one early run stopped after Rebuild with no error and didn't happen again.

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
