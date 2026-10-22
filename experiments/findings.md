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
  - At 0.01 g of leftover pull, the chain moves 0.9 m in 4.3 s. It needs the net cage, and every touch spoils the
    result.
  - The plane rotates about 90° over a parabola, so video needs the aircraft's attitude log.
  - At 1.8 g, holding just two modules out sideways takes about 1.7 N·m, above the 1.5 N·m stall torque. The chain must
    be stowed before the pull-out.
- **Air drag** costs a chain spinning at 20°/s 6–13% of its spin in 20 s (a hand estimate gives 6%, MuJoCo's model
  13%). It matters most for the spin experiments.
- **The servo model is tuned, not measured.** It barely changes net turns, but it sets joint timing and peak torque.
  Bench-test one joint before relying on torque margins.
