"""How much current the STS3032 chain draws in each setup, and the voltage its farthest servo gets, for the two ways of
powering it (../hardware/electronics.md):
  cable       6 V straight from the supply, no converters; thick tether, thicker wire through the chain. Servos get
              less than 6 V far from the feed, and can reset below about 4.8 V.
  converters  12 V on a thin chain wire, a 12 -> 6 V buck converter in every module. Every servo gets 6.0 V as long as
              each converter's input stays above about 6.3 V; the last column shows the lowest one.
Current per servo = STALL_A x |actuator torque| / its limit (a DC motor's current follows its torque). Idle draw and
spikes shorter than the 10 ms step are left out. The wires are a resistor network: a tether from the supply into m1
(and the last module with --both-ends), then CHAIN_WIRE_M of wire per module gap, + and GND both counted.
  .venv/bin/python power_check.py                       every setup, 12 modules, the cable defaults (~1 min)
  .venv/bin/python power_check.py setups/4a_walk.toml --modules 8 --tether-awg 20 --chain-awg 22
Setups whose flight-box strings tie to m12 only build with 12 modules; with fewer they're skipped.
"""
import argparse
import contextlib
import glob
import io
import os
import re
from multiprocessing import Pool

import mujoco
import numpy as np

import setup_file
from session import HERE, Session, load_model

STALL_A = 1.2           # STS3032 stall current at 6 V (listings; check the datasheet)
CUTOFF_V = 4.8          # listed minimum supply voltage; below it a servo may reset
EFFICIENCY = 0.9        # buck converter, 12 V in, 6 V out
TETHER_M = 1.5          # supply to the chain (assumed)
CHAIN_WIRE_M = 0.12     # wire per module gap: 85.5 mm spacing plus slack through the joints (assumed)
CONVERTER_AWG = (20, 22)  # tether, chain wire for the converter design: thin is enough at 12 V
OHM_PER_M = {10: 0.00328, 12: 0.00521, 14: 0.00829, 16: 0.0132, 18: 0.0210, 20: 0.0333, 22: 0.0530, 26: 0.134}


def network(n, volts, tether_awg, chain_awg, both_ends):
    """A function: the current each module draws (A) -> the voltage at each module (V)."""
    G = np.zeros((n, n))
    g = 1 / (2 * CHAIN_WIRE_M * OHM_PER_M[chain_awg])
    for k in range(n - 1):
        G[k:k + 2, k:k + 2] += [[g, -g], [-g, g]]
    supply = np.zeros(n)
    g_tether = 1 / (2 * TETHER_M * OHM_PER_M[tether_awg])
    for feed in ([0, n - 1] if both_ends else [0]):
        G[feed, feed] += g_tether
        supply[feed] += volts * g_tether
    inverse = np.linalg.inv(G)
    return lambda amps: inverse @ (supply - amps)


def check(job):
    path, a = job
    name = os.path.basename(path)
    setup = setup_file.load(path)
    setup["modules"] = a.modules
    try:
        model = load_model(**setup_file.build_args(setup))
    except ValueError:
        return f"{name:22} skipped: its flight-box strings need 12 modules"
    if setup["gravity_m_s2"] is not None:
        model.opt.gravity[:] = (0, 0, -setup["gravity_m_s2"])
    cable = network(a.modules, 6.0, a.tether_awg, a.chain_awg, a.both_ends)
    converters = network(a.modules, 12.0, *CONVERTER_AWG, a.both_ends)
    peak, cable_low, converter_low = 0.0, 6.0, 12.0
    with contextlib.redirect_stdout(io.StringIO()):  # the flight's summary; with fewer modules, the unused columns
        s = Session(model, mujoco.MjData(model), None, setup)
        s.speed = setup["speed"]
        limit = model.actuator_forcerange[:s.nj, 1]
        module = np.array([int(re.match(r"m(\d+)_", model.actuator(i).name).group(1)) - 1 for i in range(s.nj)])
        s.fly(setup["sequence"] or "", setup["move_s"], setup["hold_s"])
        while s.fly_t is not None:
            s.advance()
            amps = np.bincount(module, weights=STALL_A * np.abs(s.d.actuator_force[:s.nj]) / limit, minlength=a.modules)
            peak = max(peak, amps.sum())
            cable_low = min(cable_low, cable(amps).min())
            converter_low = min(converter_low, converters(amps * 6 / (12 * EFFICIENCY)).min())
    flag = "  RESET RISK" if cable_low < CUTOFF_V else ""
    return f"{name:22} {s.nj:2} servos  peak {peak:5.1f} A  cable {cable_low:4.2f} V{flag:12}  converters in {converter_low:5.2f} V"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("setups", nargs="*", help="setup files (default: every one in setups/)")
    ap.add_argument("--modules", type=int, default=12, help="chain modules (default 12)")
    ap.add_argument("--tether-awg", type=int, default=12, choices=OHM_PER_M, help="cable design's tether (default 12)")
    ap.add_argument("--chain-awg", type=int, default=18, choices=OHM_PER_M,
                    help="cable design's + and GND through the chain (default 18)")
    ap.add_argument("--both-ends", action="store_true", help="a second tether into the last module")
    a = ap.parse_args()
    paths = a.setups or sorted(glob.glob(os.path.join(HERE, "setups", "*.toml")))
    print(f"{a.modules} modules. Cable: 6 V, {a.tether_awg} AWG tether, {a.chain_awg} AWG chain wire, fed at m1"
          f"{' and the far end' if a.both_ends else ''}. Converters: 12 V, {CONVERTER_AWG[0]} AWG tether, "
          f"{CONVERTER_AWG[1]} AWG chain wire. Peak current is at 6 V; voltages are the lowest over the whole flight.")
    with Pool() as pool:
        for line in pool.imap(check, [(p, a) for p in paths]):
            print(line)
