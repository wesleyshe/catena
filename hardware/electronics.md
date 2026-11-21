# Electronics: the STS3032 flight build

How the STS3032 chain is controlled and powered, what's been ordered, and the sim numbers behind the choices.
Worked out with Wesley (Claude Code), 2026-10-17 to 2026-11-07. Status and history are in `AGENTS.md` and `LOG.md`.

## Decided
- **Servos:** pitch STS3032-C036 (double shaft, so the yoke is held on both sides as on the AX-12A); yaw
  STS3032-C001 (single shaft). 6 V parts: stall 0.44 N·m at 1.2 A, 12-bit encoder over 360°. Never give a servo 12 V.
- **Controller:** one Feetech FE-URT-1 USB adapter for the whole chain (Mini-USB; use its 3-pin TTL port). Its own
  power terminal is only for testing single servos at 6 V. It does the U2D2's job.
- **Software:** a Python script on the laptop with Feetech's SDK (`pip install feetech-servo-sdk`, module
  `scservo_sdk`; STS servos use its `sms_sts` part). Feetech's FD tool is Windows-only.
- **The bus:** three wires (power, GND, DATA) run through every module, and every servo is spliced onto them. Each
  servo has its own ID; a sync-write message about every 10 ms carries a target for every servo and each picks out
  its own. Servos report position, load, voltage and temperature over the same wire.
- **First build, ordered 2026-11-07:** 7 × C036 ($40), 8 × C001 ($40), 1 × C001 + URT debug board ($55); $655.
  - That's 8 modules plus `ee1`: pitch M1–M7, yaw M2–M8, and `m1_yaw`, the twist servo `ee1` sits on. 15 servos and
    one spare C001.
  - The full 12-module chain with `ee1` needs 11 × C036 and 12 × C001 (23 servos), plus spares.

## Not decided: how to power it
Both designs work for every experiment in the sim; they differ in what they cost elsewhere.

| | **Converters**: 12 V on the chain, a 12 → 6 V buck in every module | **Cable**: 6 V straight from the supply, no converters |
|---|---|---|
| Servo voltage | Always 6.0 V (each converter needs ≥ 6.3 V in) | 5.2–6.0 V, lowest far from the feed when many servos work |
| Supply | 12 V; ≥15 A for 8 modules at full stall, ≥20 A for 12 | 6 V, ≥20 A (experiments peak at 14.2 A; all 23 stalled ≈ 28 A) |
| Tether (+ and GND) | Thin: 20 AWG is enough | 12 AWG |
| Chain wire (+ and GND) | Thin: 22 AWG is enough | 18 AWG for M1–M6, 20 AWG for M7–M12 |
| In each module | A Pololu D36V28F6 (17.8 × 20.3 × 8.8 mm, 3.3 g) + a 220 µF capacitor | Nothing |
| Scales to 12 modules | Yes, any length | Yes for the experiments; marginal at full stall unless fed at both ends |
| Risk | Space for the converter in an 85 mm module | Stiff wire through the yaw (±180° on this build) and ±124° pitch joints, which the sim doesn't model |

**Before choosing:** build one joint with the cable bundle (two 18 AWG + one 26 AWG silicone) and twist it through
the full yaw and pitch range. If it fights the servo or springs back, use converters. If the CAD shows the converter
won't fit, use the cable.

A battery doesn't change this. A 3S LiPo (12.6 → 9.0 V) drops into the converter design as a "module 0" bolted to
M1's end, with a BMS, charger, arming plug and an ESP32 in place of the USB link. A "6 V" battery (2S LiFePO4,
5 × NiMH) swings about 7.2 → 5 V, so servo strength would follow its charge.

## The sim's numbers (`simulation/power_check.py`)
**Method:** fly each setup (as of 2026-11-07: 10 s 1.8 g phases); current per servo = 1.2 A × |torque| / its limit;
solve the wires as resistors, with a 1.5 m tether and 0.12 m of chain wire per module gap (both assumed), + and GND
counted. Idle draw and spikes under 10 ms are left out. Masses are the `feetech85` placeholders.

**Peak current at 6 V, 12 modules + ee1:** 1.9–14.2 A. The highest are `5b_rod_crawl` (14.2 A), `3a_*` (11–12 A) and
`1e_turn_across` (9.5 A), mostly at the pull-out and landing on the net and in the anchored arm's 1.8 g pull-up.

**Lowest servo voltage over all 21 setups, cable design, 12 AWG tether:**

| Chain wire | Fed at M1 | Fed at both ends |
|---|---|---|
| 20 AWG | 5.15 V | 5.75 V |
| 18 AWG | 5.39 V | 5.81 V |
| 16 AWG | 5.53 V | 5.85 V |

**All 23 servos stalled at once (27.6 A; by hand, not in any experiment):** fed at M1, 4.4 / 4.8 / 5.1 V with
20 / 18 / 16 AWG; fed at both ends, about 5.5–5.7 V.

**Converter design:** the lowest converter input is 10.7 V (`5b`, 20 AWG tether, 22 AWG chain), far above 6.3 V, so
every servo stays at 6.0 V.

**Why the thin-wire 6 V design fails:** with a 20 AWG tether and 22 AWG chain wire, the far servo drops below 4.8 V in
10 of 21 setups (to 3.6 V); with 26 AWG, 15 of 21. At 14 A the thin tether alone loses about 1.4 V.

**The first build (8 modules + ee1, 15 servos):** only the 7 setups without flight-box strings build at 8 modules (the
strings tie to m12). Peak 7.2 A (`4a_walk`); with the thin 20 AWG tether and 22 AWG chain, lowest 4.99 V. Fed at M4
instead: 5.21 V. Three zero-g sequences played flat on a table at 1 g (`fold_sequence`, `reorient_loop`,
`roll_loop`): 2–5.5 A, lowest 5.22 V. All 15 stalled: 18 A, about 3.5 V at the far servo, so a jam or a power-up with
every servo switched on at once can brown out the far end without converters.

**What low voltage does:** torque and speed scale roughly with voltage (5 V ≈ 83%). Below about 4.8 V (the listed
minimum) a servo can reset or cut its motor; nothing is damaged. The sim assumes 6 V everywhere, so sag makes the real
fold drift from the prediction. Log every servo's reported voltage: below about 5.5 V, that run was power-limited.

To re-run: `cd simulation && .venv/bin/python power_check.py` (every setup, ~1 min); `--modules 8`,
`--tether-awg 20 --chain-awg 22` and `--both-ends` change the build. Its constants (stall current, wire lengths) are at
the top.

## Parts lists (besides servos and the URT)
**Converter design, first build (8 modules + ee1), every servo able to stall at once:**

| Part | Qty | Notes |
|---|---|---|
| Bench supply, 12 V, ≥15 A, adjustable current limit | 1 | All 15 stalled ≈ 10.2 A at 12 V |
| Pololu D36V28F6 (12 → 6 V, 2.7 A) | 8 + 1 spare | $21.11 each at 5+; each feeds ≤ 2 servos (2.4 A stalled) |
| Capacitor, 220 µF, 16 V, low-ESR | 8 + spares | Across each converter's 6 V output |
| Resettable fuse (PTC), ~2 A hold | 8, optional | On each converter's 12 V input; one shorted module trips alone |
| Main fuse, 15 A + inline holder | 1 | On the supply's + lead |
| Terminal block, 3-way + 1000 µF 35 V capacitor | 1 each | Where the supply, the URT and the tether join |
| Wire | | Tether 18 AWG (+, GND); chain 20 AWG (+, GND); DATA 26 AWG |

**Cable design, 12 modules:** a 6 V ≥20 A supply with current limit, a 20 A main fuse, a terminal block, a 12 AWG
tether (+, GND), 18 AWG chain wire for M1–M6 and 20 AWG for M7–M12, DATA 26 AWG. Optionally a second tether to M12.

**Both:** Mini-USB cable (+ USB-C adapter), multimeter, soldering iron, solder, heat-shrink.

**Not decided yet:** the IMU and onboard data logging the proposal needs, the battery, and the end effectors' own
hardware.

## Wiring rules
```
laptop --USB--> URT-1 --DATA, GND--+
supply --fuse--> +, GND -----------+--> terminal block --> tether --> M1 --> M2 --> ... (+, GND, DATA pass through)
converter design, in each module:  +12 V, GND --> buck --> 6 V, GND --> that module's servos; DATA --> each servo
cable design, in each module:      each servo's 6 V, GND and DATA spliced straight onto the three chain wires
```
- The URT's lead carries DATA and GND only: cut its power wire so the chain's supply can't reach it.
- All grounds are joined (supply, URT, every converter and servo).
- Feed power at the M1 end, the anchored end in the setups, so the tether runs along the anchor instead of tugging
  the moving chain. Feeding at the middle helps little (the stall tests load the anchored end).
- Splice at the module, never through a servo's own lead: it's sized for one servo's current.
- No perfboard is needed: solder wires to the converter's pads, put the capacitor across 6 V and GND there, and
  heat-shrink it. JST-GH plugs are rated about 1 A per pin, too small for the power pins.

## IDs and bring-up
- Every servo ships as ID 1. Set each one alone on the URT, then label it. Number them in the sequence CSV's column
  order: `m1_yaw` (ee1's twist) = 1, `m1_pitch` = 2, `m2_yaw` = 3, …, `m8_yaw` = 15 (the 12-module chain: … `m12_yaw` =
  23). Record IDs, turning directions and zero offsets in `spec.toml` (its last comment is the placeholder).
- Position = 2048 + degrees × 4096 / 360, plus that servo's zero offset.
- Order: set IDs at 6 V on the URT; set the supply and its current limit; with no servos plugged in, check every
  converter (or the cable's far end) reads 6 V; then connect the servos, switching them on one module at a time.
- For desk tests without converters, set a torque limit (50–70%) in each servo.

## To check on the test servos
- Supply voltage limits (listings say 4.8–6 V or 4–7.4 V); with 7.4 V allowed, the cable design could run at ~6.3 V.
- Stall current (1.2 A from listings), the connector and whether there's one lead or two, backlash at the horn.
- The C036's length: listed 32 × 12 × 27.5 mm against the C001's 23.2 × 12.1 × 28.5 mm.

## Sources
Listings read via search results, 2026-10-17 to 2026-10-25: [STS3032-C036, RobotShop EU](https://eu.robotshop.com/products/feetech-6v-45-kg-360-magnetic-encoders-double-shaft-serial),
[STS3032, Global Sources](https://www.globalsources.com/product/rc-micro-servo_1208286292f.htm),
[FE-URT-1, RobotShop UK](https://uk.robotshop.com/products/fe-urt-1-usb-to-ttl-485-bus-programmer),
[Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/bus-servo-adapter-a.htm) (takes 9–12.6 V and passes it
to the servo plug, so not for 6 V servos), [Pololu D36V28F6](https://www.pololu.com/product/3783).
