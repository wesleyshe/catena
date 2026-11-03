# Experiments

These are the microgravity experiments we're designing, and what the simulation can predict for each. They come from
`../docs/CATENA Proposal.docx`, which lists more than we expect to fly; scope will narrow during design. All three share
one chain and one set of electronics.

**Sequences vs experiments:**
- A **sequence** (`../sequences/`) is a scripted move: joint angles over time.
- An **experiment** is a question we'll answer in flight. It records:
  - the prediction
  - the procedure: which sequences, at which gravity levels
  - the success criteria
  - later, the flight data next to the prediction

Measured sim results so far are in `findings.md`.

## A. Folding
- **Questions:**
  - Does a chain longer than its 1 g stall length complete its fold in free fall? On the bench it stalls partway.
  - Fold completion vs chain length at 1.8 g, 1 g, 3/8 g, 1/6 g and 0 g, compared with the prediction
    N_max ~ g^(−1/2).
  - Orientation change per fold (from an onboard IMU and video), compared with a pre-flight multibody model.
- **Success, in tiers:**
  1. A chain that stalls on the ground completes its fold.
  2. Its orientation ends within tolerance of the prediction.
  3. Two fold orders reach the same shape at measurably different orientations.
- **What the sim does now:**
  - Any gravity level: the **Gravity** box or `--gravity`.
  - Zero-g folds that conserve momentum (a numerical check: the centre of mass stays put to 0.005 mm).
  - The leftover spin at release: **Spin** with an axis, or `--spin X Y Z`.
  - The IMU's view: its module's 3D attitude and rotation rate in the log (`imu_*` columns) and the `IMU turn` readout.
  - Cabin air drag (`--air`), and a flag when a sequence is longer than one parabola (`--speed` to fit).
  - **The flight box and a whole parabola:** a setup with a `[flight]` table puts the chain in the acrylic box (sized
    from the A310 cabin, `flight_space.md`) with a net across it, held by strings that let go at zero g; **Fly a
    parabola** (or `play.py --fly`) runs 1.8 g, zero g with the leftover g, 1.8 g. Setup `1d_flight_turn`; results in
    `findings.md`, "The flight box".
  - `reorient_loop.csv` turns the chain about 7.4° per loop with zero angular momentum, about the vertical only.
  - `roll_loop.csv` turns it about its long axis, about 32° per loop. Turning in 3D needs a twist as well as pitch
    folds.
  - What moves that prediction, and by how much, is in `findings.md` under "Is the sim good enough for flight?".
  - **A held end and the stall length:** an anchor end effector on a wall (setups `2b_stall_*`), and
    `../simulation/stall_sweep.py`. The longest chain that stands out level is 3 / 4 / 6 / 12+ modules at Earth /
    Mars / Moon / zero g (`findings.md`, "The class simulations").
- **Not in the sim yet:**
  - **Prediction ranges.** Repeated runs over release spin, masses and servo settings, so success tier 2 has a
    tolerance.
  - **The aircraft's own rotation:** its ~90° pitch-over (for video, and the spin the chain keeps when it's let go).
    The 1.8 g phases and the leftover g are in (above).
- **Flight plan must cover:**
  - Recording the IMU's rate at release. Even 1°/s cancels the whole 30° turn.
  - Stowing the chain before the 1.8 g pull-out. Two modules held out sideways already exceed the servo torque.
  - Sequences inside one parabola, about 20 s.

## B. Manipulation
- **Question:** with one end anchored, can the chain act as an arm?
  - Does it hold a commanded pose at all?
  - Tip repeatability and available tip force.
  - Reach to a target behind an obstruction.
- **Hardware:** this needs an anchored fixture, so it's a separate rig. A stands alone if only one flies.
- **What the sim does now:** an anchor end effector holds one end to a wall, the floor or a rod, with the clamp's force
  and torque in the readouts and the log (setups `2a_arm`, `2c_let_go`). Suction cups and thrusters too (setups `3a_*`,
  `4a`, `4b`, `5a`).
- **Not in the sim yet:**
  - Obstacles.
  - A tip-force readout.

## C. Learned body representation
- **Question:**
  - The chain explores combinations of joint motions and records the whole-body response, building a library that
    links actions to consequences.
  - Then damage or alter it, and test whether it can identify the change and find a new strategy with the remaining
    modules.
- **What the sim could offer:** a fast headless runner (`play.py --headless`) to build a simulated library before
  flight. It can also stand in for damage by freezing a joint.
- **Not in the sim yet:** an exploration runner and a damage option.

## Adding an experiment
When an experiment gets concrete, give it a folder here (e.g. `A-folding/`) containing:
- a README with:
  - the question
  - the prediction, with the exact sim command that produced it
  - the procedure
  - the success criteria
  - later, the flight results
- the sequences it uses, placed in `../sequences/` and referenced by name

Log changes in the root `LOG.md`.
