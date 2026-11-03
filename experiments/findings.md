# Findings from the sim

These are answers to physics questions, most of them from the course professor (forwarded by Wesley, 2026-10-17 to
22). Each one was answered with a simulation run, not theory alone. All runs used 12 × original14 in zero gravity unless
stated otherwise.

**These are measurements in the sim, which makes them predictions for the real chain.** They hold for the numbers in
`../hardware/spec.toml` at the time, and the masses there are still placeholders. Nothing here has been checked against
the real chain yet.

To add one, write the question, the run (sequence, joint and angle, or command), and the measured numbers.

## A joint splits its turn by inverse moment of inertia (mass × distance²); length dominates
| Joint, commanded to | Side before the joint | Side after |
|---|---|---|
| m3_pitch 90° | −82.4° | +7.6° |
| m3_pitch 60° | −54.1° | +5.9° |
| m6_pitch 90° | −51.1° | +38.9° |
| m6_pitch 60° | −33.7° | +26.3° |
| m6_pitch 60°, modules 1–6 twice as heavy | −30.6° | +29.4° |
| m6_pitch 60°, modules 7–12 twice as heavy | −36.4° | +23.5° |
| m1_pitch 60° | module 1 spins in place | the rest moves 0.01° |

- The relative angle is always what was commanded.
- The centre of mass doesn't move.
- Doubling one side's mass barely moves the split.

## Turning a tip link turns the whole system a bit, then everything stops
- **m11_pitch 90°:** the rest of the chain turns 2.2°.
- **A full-speed −90° → +90° swing:**
  - The rest turns 4.5°.
  - It peaks at 12°/s during the swing, then drops to 0.00°/s.
- **Internal motions can't create a lasting spin.** "Centrifugal force keeps you rotating" is wrong when you start
  from rest.

## Folding changes the spin rate only if the chain is already spinning (L ≠ 0)
- **Spinning at 20°/s straight:** it spins at 31.9°/s folded into an L, and at 20°/s again when unfolded.
- **L stays constant throughout,** like a skater pulling their arms in.
- **The Spin button supplies the L ≠ 0.**

## Turning with zero angular momentum (the "falling cat")
- **One joint out and back gives no net turn,** whatever the speeds.
  - Fast out and slow back gave −0.15° per cycle.
  - That's a small real turn, not noise: the servos flex, so the joint doesn't retrace exactly the same path back.
    Backlash in the real gears will change it.
- **A loop through two joints turns the chain about 7.4–7.8° per cycle,** whatever the speed, as long as the servos
  keep up.
  - Faster than about 3 s per loop they lag, and the turn grows; see "Is the sim good enough for flight?" below.
  - The loop is m4 out, m8 out, m4 back, m8 back.
  - Reversing the loop reverses the turn.
  - `../sequences/reorient_loop.csv` runs 4 loops, about 30°. At its default timing that takes 65 s; at `--speed 4` it
    takes 16 s and still turns about 31°.
- **This is the basis of experiment A's "two fold orders, two orientations" test.**

## One fold vs. loops of folds, in 3D (2026-10-22)
A team question (Sergio): does one fold reorient the chain, or does it take more? Measured from rest with the sim's IMU
output (module 1's 3D attitude), at move 1.5 s, hold 0.5 s.

| Move | Lasting turn of the whole chain | About |
|---|---|---|
| One fold (m6 pitch to 90°), held | 51.3°, only while folded | vertical |
| One fold, out and back | 0.02° | |
| Loop of two pitch folds (m4, m8 out, then back) | 7.4° per loop | vertical only |
| Loop of a pitch fold and a twist (m4 pitch, m8 yaw) | 31.8° per loop | the chain's long axis |
| The same loop in reverse order | −31.8° | long axis, the other way |
| Pitch + twist loop, 4 times | 127° | long axis |

- **One fold can't reorient the chain.** It turns only while the fold is held, and undoing the fold undoes the turn.
- **A lasting turn needs a loop through at least two joints,** because a loop doesn't retrace its own path.
- **Turning in 3D needs joints that point different ways.**
  - With the chain lying flat (`CHAIN_ROLL = 0`), all the pitch axes are parallel, so pitch-only loops can only turn it
    about the vertical.
  - Adding a twist (yaw) turns it about the long axis.
  - Loops about different axes should be able to reach any orientation.
- **The CG moved 0.0000 mm in every case,** and angular momentum stayed around 1e-10. Zero momentum keeps the CG fixed
  and rules out a lasting spin, but it doesn't fix the orientation.
- **Caveat:** the twist loop's 31.8° depends on how the mass is spread around the chain's long axis. That spread is
  small, and it's the least certain number in the sim until the modules are weighed.
- **`../sequences/roll_loop.csv`** runs 2 pitch-plus-twist loops: 63.6° at default timing (33 s), and 63.4° at
  `--speed 2` (16.5 s, which fits one parabola).

## An internal rotor turns the body the opposite way for as long as it spins
- **Module 12 spun at 82°/s with `--slip-ring`:** modules 1–11 turn steadily at −7.3°/s.
- **Overall spin stays 0,** and both stop when the rotor stops.
- **This is how a reaction wheel works.**
- The motor rotors' own spin, which isn't modelled, adds roughly ±3% uncertainty to this result.

## Where it turns
Everything turns about the centre of mass, which stays fixed: `COM drift` reads 0.000 mm throughout.

## Is the sim good enough for flight? (review, 2026-10-22)
An outside reviewer read the code cold, and separate sim runs tested flight conditions one at a time. All runs used
`reorient_loop.csv` (4 loops) at flight timing (move 1 s, hold 0.25 s: 20 s, turning −30.3° from rest).

**Sound:**
- Angular momentum is conserved to 1e-10, and results don't change at half the timestep.
- The inertia, Spin and joint axes are correct.
- A uniform leftover pull in the cabin doesn't change the turn: at 0.001 g and 0.01 g it matched zero g to within
  0.01°, up to the moment the chain touched something.

**What moves the prediction, and by how much:**

| Change | Turn after 4 loops | Shift |
|---|---|---|
| None (zero g, let go at rest) | −30.3° | |
| Let go spinning at 0.5°/s | −15.2° | +15° |
| Let go spinning at 1°/s | 0.0° | +30° (cancelled) |
| Let go spinning at 5°/s | +121.5° | +152° |
| CAD masses (281 g base, 24 g arm): everything heavier together | −30.8° | −0.4° |
| +150 g battery in each end module | −33.0° | −2.6° (9%) |
| +300 g battery in the middle module | −33.0° | −2.7° (9%) |
| +25 g in every module | −30.5° | −0.1° |
| Cabin air drag, let go at rest | −29.0° | +1.4° (5%) |
| Any one servo setting ±50% (`kv`, armature, friction, stiffness) | −29.1° to −31.1° | ≤ 1.2° |
| Battery sag to 10 V (torque and speed × 0.83) | −30.6° | −0.3° |
| The same loops faster: 11 s in total | −36.8° | −6.5° (servos lag) |
| 6 s in total | −45.7° | −15° (servos saturated 46% of the time) |

**What this means for the flight:**
- **The spin at release dominates.**
  - A chain let go from a holder fixed to the cabin starts out turning with the plane, which pitches over at about
    3–6°/s during a parabola (g·cos γ / V, an estimate).
  - Record the IMU's rate at release, and predict from that measured rate (`--spin X Y Z`), not from rest.
- **Where the mass sits matters; total mass doesn't.** Weigh the modules in flight configuration and enter them in
  `module_g`.
- **Stay within the servos.** Keep sequences slower than about 3 s per loop and inside one parabola (`--speed`, and the
  `sequence` readout flags anything longer).
- **The cabin:**
  - At 0.01 g of leftover pull, the chain moves 0.9 m in 4.3 s. It needs the flight box, and every touch spoils the
    result.
  - The plane rotates about 90° over a parabola, so video needs the aircraft's attitude log.
  - At 1.8 g, holding just two modules out sideways takes about 1.7 N·m, above the 1.5 N·m stall torque. The chain must
    be stowed before the pull-out.
- **Air drag** costs a chain spinning at 20°/s 6–13% of its spin in 20 s (a hand estimate gives 6%, MuJoCo's model
  13%). It matters most for the spin experiments.
- **The servo model is tuned, not measured.** It barely changes net turns, but it sets joint timing and peak torque.
  Bench-test one joint before relying on torque margins.

## The class simulations (end effectors, 2026-10-29)
Six simulations for class, each a setup in `../simulation/setups/` (a demo button, or Load then Play, in the control
window). End
effectors are extra modules on the ends: an anchor (a clamp), a suction cup or a thruster (five nozzles). Their
masses and limits are placeholders. All are in zero g unless the table says otherwise; numbers are headless runs.

| Simulation | Setup | What it shows in zero g | Measured |
|---|---|---|---|
| 1. Free float | `1a_turn` | turning with no push (fold loops at flight timing) | turned −30.3° in 20 s |
| | `1b_spin_0.5`, `1b_spin_1` | the spin at release swamps it | −15.2°; 0.0° |
| | `1c_roll` | rolling about its own length (a fold plus a twist) | IMU turn 63.4° in 16.5 s |
| 2. One end anchored | `2a_arm` | an arm on a wall; the clamp takes the reaction | peak clamp load 4.0 N |
| | `2b_stall_1g` / `_mars` / `_moon` / `_0g` | why zero g matters: m1_pitch lifting the hanging chain | lifts 12.6° / 24.4° / 44.3° / all the way (level) |
| | `2c_let_go` | let go mid-swing: it keeps the momentum the wall was taking | drifts off at 150 mm/s, spinning |
| 3. Thrusters | `3a_straight` / `_U` / `_Z` / `_L` | a 2.5 s half-throttle burst of both end nozzles, then coasting | 0 mm; glides 0.93 m (turns 1°); spins 8.2°/s (moves 0.16 mm); 0.66 m and 43° |
| 4. Suction cups | `4a_walk` | an inchworm walk; a foot lets go only when the other holds | 1.10 m in two steps, 19 s, no pops |
| | `4b_walk_let_go` | the back foot lets go too early | floats off the floor |
| 5. Pole climb | `5a_pole_climb` | the bare chain wrapped around a Ø100 mm pole climbs it by squeezing, like a snake | 394 mm up in 19.3 s; swings ±50° around the pole |
| | `5b_rod_crawl` | clamps crawling along a thin handrail | 1.115 m along the rod in 19 s; peak clamp load 3.7 N |
| 6. Skater | `6a_skater` | spinning at 20°/s: faster folded, back to 20°/s straight | 31.9°/s in the L (section 6 of `verify.py`) |

**Stall length (experiment A's number).** `../simulation/stall_sweep.py` hangs chains of 2–12 modules from a wall by
one end and tells them to stand straight out. The longest that ends within 10° of level:

| Gravity | Longest chain that stands level | Hand estimate (N(N+1)/2 · m·g·pitch ≤ 1.5 N·m, 230 g modules) |
|---|---|---|
| Earth, 9.81 m/s² | 3 modules | 2 |
| Mars, 3.71 | 4 | 4 |
| Moon, 1.62 | 6 | 7 |
| zero g | all 12 | any |

The other joints sag too, so a 12-module chain gets m1_pitch further (12.6° at 1 g) than the straight-chain estimate
(4°). Earth→Moon doubles the stall length, close to N_max ~ g^(−1/2) (×2.5).

**A suction cup has to resist tipping, not pulling.** Walking, each cup feels only ~1 N of pull, but swinging the rest
of the chain tips the standing foot. A 40 mm, 20 N cup (the placeholder) peels and the chain pivots off the floor;
40–70 mm cups fail the same way; an 80 mm, 60 N cup walks. That's roughly pull × radius ≈ 2.4 N·m of hold against
tipping: the number to size a real cup by.

**The side nozzles roll the chain a little.** Every nozzle's push passes through the chain's axis, but each module's
centre of mass sits ~4.8 mm off it (the servo block isn't centred in the shell). So the nozzles along the end's ±y turn
it with no roll, while those along ±x also roll it, ~12.8°/s after 2 s of 0.1 N, because a straight chain is very easy
to roll about its length. Rolling on purpose still needs the fold-and-twist loop (`1c_roll`).

**Letting go only carries momentum if the chain is moving.** Sequences stop at every row, so `2c_let_go` swings fast
(0.5 s per move): the servos are still catching up when the clamp opens, and that motion is what it drifts off with.

**Wrapped around a pole, the chain grips by squeezing: no suction cups or clamps needed.** Every pitch the same and
every yaw the same make a coil; around a Ø200 mm pole that's 47° of pitch per joint (limit 124°), 1.5 turns for 12
modules. Wrapped just to the pole's shape it slides off at 2 N. Told to bend 8° tighter, 10 shells press on with 59 N
in all and it holds a 20 N push along the pole (12 mm of give, no slip), with friction 1. The cost: the squeeze holds the
servos near stall (1.3 of 1.5 N·m) for as long as it grips, which a real AX-12A won't sustain; with plastic shells
(friction ~0.3) it would hold ~20 N, still far above the 1–4 N it meets in zero g. `5a_pole_climb` uses this on a Ø100 mm
pole: two 4-module coils take turns squeezing while the middle stretches (from 315 to 499 mm along the pole), ~13 cm per
step; a quarter of the designed stretch is lost to slip.

## The flight box (2026-11-03)
`../simulation/setups/1d_flight_turn.toml`, flown with `play.py --fly` (or **Fly a parabola**): the 8.5 cm flight build
(`feetech85`, scaled from the AX-12A chain: Ø84.9 mm shells 0.6 mm apart, STS3032, 0.81 kg) in the 2.5 × 1.3 × 1.9 m
box from `flight_space.md`, through one A310 parabola (1.8 g, 22 s of zero g with the ITF's leftover g, 1.8 g). Four
strings each hold m1, m5, m8 and m12 still in the middle, running to the box's top and bottom edges; they let go at
T+1 s (T = 0 when zero g begins), and the turn sequence (`reorient_loop.csv`, 20.25 s) starts at T+1.5 s.

- **Holding through 1.8 g is easy for the servos.** The chain moves 0.36 mm at most and the strings pull up to 3.5 N,
  but the servos carry 0.01 N·m of their 0.44: with the pitch axes vertical, gravity can't bend any joint, so the
  hinges take the load.
- **The strings must be angled.** With strings only straight up and down (the first try), the leftover g swung the
  chain about 10 mm sideways and it left the strings at 17 mm/s. Fanned out to the box's edges they hold it every way,
  and the release is clean: 2 s after letting go, with no jitter, it drifts 0.1 mm/s and spins 0.000°/s.
- **The turn survives the real cabin.** With the leftover g, the 30° turn comes out at −30.1° (−30.3° in perfect zero
  g). The chain wanders 0.66 m from where it was let go and comes within ~220 mm of the box, without touching it. That's one jitter
  pattern (seed 1); other seeds will touch sooner or later.
- **Size doesn't change the turn.** The 8.5 cm chain turns the same −30.3° as the 14 cm one: the turn follows the shape
  and where the mass sits, not the size.
- **The landing.** At the pull-out it lands on the net at 2–3 m/s, depending on where it has drifted, and sinks
  48–72 mm of the net's 200 mm. Both net numbers are placeholders (20 mm of sag at 1 g, 200 mm above the box floor).
- **Not modelled yet:** strings letting go at slightly different times, the plane's own rotation (~3–6°/s, which the
  chain keeps when it's let go), and a real net's cloth. The sim's net matches a net's spring for the landing but
  sits only a few mm deep at rest.
