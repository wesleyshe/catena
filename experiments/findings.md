# Findings from the sim

These are measured answers to physics questions, most of them from the course professor (forwarded by Wesley,
2026-10-17 to 22). Each one was answered with a simulation run, not theory alone. All runs used 12 × original14 in zero
gravity unless stated otherwise.

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
  - That's noise from servo flex.
- **A loop through two joints turns the chain about 7.4–7.8° per cycle,** almost regardless of speed.
  - The loop is m4 out, m8 out, m4 back, m8 back.
  - Reversing the loop reverses the turn.
  - `../sequences/reorient_loop.csv` runs 4 loops, about 30°.
- **This is the basis of experiment A's "two fold orders, two orientations" test.**

## An internal rotor turns the body the opposite way for as long as it spins
- **Module 12 spun at 82°/s with `--slip-ring`:** modules 1–11 turn steadily at −7.3°/s.
- **Overall spin stays 0,** and both stop when the rotor stops.
- **This is how a reaction wheel works.**

## Where it turns
Everything turns about the centre of mass, which stays fixed: `COM drift` reads 0.000 mm throughout.
