# Flight space: how big a box fits (Novespace A310 ZERO-G)

From `../docs/ITF-2025-01-Interfaces-EN.pdf` (the A310 interface document, "ITF", revision 2025-01). Heights are above
the floor with the seat-track studs counted (ITF Figure 4); positions across are from the cabin's centreline.

## The limits
| Limit | Value | Source |
|---|---|---|
| Test area | 19.6 m long, 5 m wall to wall, ≈200 m³ | ITF §2.4.1 |
| Centre aisle | Must stay clear: an escape corridor at least 508 mm wide above 635 mm (381 mm below) | ITF §4.1, CAB-05 |
| Ceiling | 2.25 m over the middle; 2.18 m at the edge of the centre ceiling, ~1.35 m out; 1.97 m under the overhead ledge, ~1.40–1.80 m out; 1.88 m at ~1.96 m; the handrail at 1.59 m, ~2.1 m out; 1.14 m at the outer track | ITF Figure 4, §2.4.6 |
| Floor tracks | At ±0.450, ±0.953, ±1.456, ±1.959 and ±2.462 m; attach points every 25.4 mm along them | ITF CAB-03, §4.1 |
| Loading door | Everything must go through it: 1.80 m high, 1.06 m wide (960–1120 mm, depending on the height) | ITF §2.3, ITF-01, Appendix 1 |
| Along the cabin | Not across a rotating flexible joint (9.17 m and 15.53 m from the front of the test area); nothing between 16.30 and 16.59 m | ITF CAB-04 |

How much of the 19.6 m one experiment gets is up to Novespace; these documents don't say. The Skinsuit safety data
package (`../docs/Example_ESDP-*`) used about 2.5 m along one side.

## A box on one side of the aisle
The box's aisle face sits on the first track, 0.45 m out, so the aisle stays clear.

| Box (inside) | Across | High | Along the cabin | Volume |
|---|---|---|---|---|
| **The sim's default** (`box_mm` in `../hardware/spec.toml`) | 1.30 m (0.45 → 1.75 m out) | 1.90 m (under the 1.97 m ledge, 7 cm spare) | 2.0 m | 4.9 m³ |
| Longer, if Novespace gives the length | 1.30 m | 1.90 m | 2.5 m | 6.2 m³ |
| Tall | 0.90 m (0.45 → 1.35 m out) | 2.15 m (under the 2.18–2.25 m centre ceiling) | 2.0 m | 3.9 m³ |
| One piece, through the door | 1.0 m | 1.75 m | 2.0 m, if it can turn from the door into the cabin | 3.5 m³ |

- **People can walk past.** With the box outside the first track, the aisle between the two first tracks is 0.9 m
  wide, against the 0.508 m the escape corridor needs above 635 mm. That holds as long as the experiment across the
  aisle also stays outside its first track. There's no walkway on the wall side: the wall slopes in over the box's
  outer edge.
- **2.5 × 2.5 × 2.5 m doesn't fit.** The ceiling is 2.25 m at most, and only over the aisle. One side has about 2 m
  of floor, but only about 1.3 m of it has 1.9 m of headroom.
- **Getting it on board:** 2.0 × 1.3 × 1.9 m can't go through the 1.06 × 1.80 m door in one piece, so it has to be a
  kit assembled in the cabin. Check with Novespace that they accept that. For example, a frame of 2 m aluminium
  extrusions with acrylic panels no wider than 1 m: a 1.0 × 1.9 m panel goes through tilted (the door's diagonal is
  about 2.1 m). In 6 mm acrylic all six faces weigh about 130 kg before the frame, so thinner sheet or netting on some
  faces is worth a look. The structure also has to meet Novespace's load cases (REF-02 in the ITF).
- **Someone has to reach in between parabolas** to tie the strings back on, so the box needs a door or a panel that
  opens (the sim draws one in the aisle-side panel, 0.8 × 1.4 m).
- **The net sits 300 mm above the box's floor** (`net_height_mm`), on its own frame. That room is the buffer for a
  landing: the sim's landings sink 30–100 mm into a net that sags 20 mm at 1 g, so a falling chain never reaches the
  floor. It leaves 1.6 m above the net, and the strings hold the chain in the middle of that.
- **A walking deck:** the net covers the floor, so the suction walk stands on a rigid deck laid over the net.
- **Along the cabin is the cheap direction.** The chain lies along it in the sim, and length costs no headroom.

## The g levels the sim uses (`[flight]` in the spec)
| What | Value | Source |
|---|---|---|
| Hypergravity | Up to 1.8 g, about 23 s before and after each parabola | ITF §3.10 |
| The plane's pitch | Nose up ~45° as zero g begins, over the top, nose down ~45° as it ends: the box turns ~90°, fastest (~5°/s) at the top. 45° is typical; the attitude is in Novespace's flight data | ITF §3.10 (the data) |
| Microgravity | About 22 s | ITF §3.6, §3.10 |
| Leftover g ("g-jitter") | Normally ±0.02 g vertical, ±0.015 g along and across (the measured table gives ±0.03 / ±0.02 / ±0.01 g) | ITF §3.10, Table 2 |
| Free-floating objects | Reach the cabin walls in 4–5 s | ITF §3.10 |
| Acceleration record | 10 Hz, 0.001 g resolution, given after the flight. Novespace advises your own accelerometer for accurate, time-synced data | ITF §3.10 |
| Cabin | 825 hPa (air ≈1.0 kg/m³), 17–20 °C in flight, humidity up to 15 % | ITF §3.2–3.4 |

In the sim, a setup with a `[flight]` table puts the chain in the box (every setup in `../simulation/setups/` has one),
and Play runs this profile, the plane's pitch included. What it found is in `findings.md`, "The flight box".
