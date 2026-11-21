"""What the model is built from: the real chain's numbers (../hardware/spec.toml, read once), the folders the sim reads
and writes, and v(), which writes numbers into the model's XML. Stdlib only, like everything the control window imports.
"""
import os
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))  # simulation/
REPO = os.path.dirname(HERE)
SPEC_FILE = os.path.join(REPO, "hardware", "spec.toml")
SEQ_DIR = os.path.join(REPO, "sequences")  # at the repo root, shared with the robot software
SETUP_DIR = os.path.join(HERE, "setups")
with open(SPEC_FILE, "rb") as fh:
    SPEC = tomllib.load(fh)  # the chain as built; hardware owns it, and the robot software reads it too
HW, SERVO, MASS, FLIGHT = SPEC["chain"], SPEC["servo"], SPEC["mass"], SPEC["flight"]


def v(*xs):
    """Numbers as one XML attribute: v(0, 0.5, 1) is "0 0.5 1"."""
    return " ".join(f"{x:.6g}" for x in xs)
