"""Drive the chain: interactive viewer, fold-sequence playback, and logging.

  mjpython play.py                                  viewer: Control-panel sliders + keyboard
  mjpython play.py --seq ../sequences/fold_sequence.csv       viewer, playing the sequence
  .venv/bin/python play.py --seq ../sequences/fold_sequence.csv --headless    no viewer, run and log only
  .venv/bin/python play.py --setup setups/1a_turn.toml --headless   a whole run from a setup file (setups/); one with
      [flight] flies a whole parabola: held by strings through the 1.8 g pull-up, let go at zero g, onto the net
  mjpython play.py --gravity 9.81                   start with Earth gravity: the chain drops onto the floor
  .venv/bin/python play.py --seq ../sequences/reorient_loop.csv --headless --speed 4 --spin 0 0 1 --air
      flight-like: the sequence 4x faster (to fit a parabola), let go with 1 deg/s of spin, cabin air
The control window (panel.py, or double-click 'Chain Sim.command') starts this with --panel and sends it commands.
This file is only the command line: the run is session.py, the viewer window viewer.py.

Keyboard (viewer): PageUp / PageDown select module, Up / Down nudge its pitch,
Right / Left nudge its yaw, Space pauses / resumes the physics.
The selected module turns orange.
On a Mac the viewer needs mjpython (it ships in .venv/bin); --headless works with plain python.
"""
import argparse
import os

import mujoco

import setup_file
from flight import Parabola  # noqa: F401 (play.py's names, for verify.py and anyone's scripts: from play import ...)
from sequence import LOG_DT, actuator, joint_servos, modules, read_sequence, target  # noqa: F401
from session import HERE, PARABOLA_S, Session, load_model
from viewer import Commands, run_viewer  # noqa: F401


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", help="model XML (default: built fresh from make_chain.py and ../hardware/spec.toml)")
    ap.add_argument("--seq", help="fold sequence CSV (degrees)")
    ap.add_argument("--move", type=float, default=3.0, help="seconds to move between rows")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold each row")
    ap.add_argument("--log", default=os.path.join(HERE, "logs", "run_log.csv"), help='log file; "" for no log')
    ap.add_argument("--headless", action="store_true", help="no viewer; needs --seq")
    ap.add_argument("--gravity", type=float, help="m/s^2 downward, overriding the model (Earth: 9.81)")
    ap.add_argument("--speed", type=float, default=1.0, help="play the sequence this many times faster (e.g. 4)")
    ap.add_argument("--playback", type=float, default=1.0,
                    help="viewer: show the simulation this many times faster than real time; the physics doesn't change")
    ap.add_argument("--spin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="start spinning at this rate (deg/s) about world x, y, z: the leftover spin when it's let go")
    ap.add_argument("--air", action="store_true", help="cabin air drag on the shells (when building the model)")
    ap.add_argument("--setup", help="setup file (setups/*.toml): the model and the settings above; flags given here win")
    ap.add_argument("--panel", type=int, help=argparse.SUPPRESS)  # port of the control window
    a = ap.parse_args()
    try:
        setup = setup_file.load(a.setup) if a.setup else None
    except (OSError, ValueError) as e:  # say what's wrong with the file, without a traceback
        raise SystemExit(f"can't use that setup: {e}")
    if setup:  # the setup's values become the defaults, so flags on the command line still win
        ap.set_defaults(move=setup["move_s"], hold=setup["hold_s"], speed=setup["speed"], gravity=setup["gravity_m_s2"],
                        air=setup["air"], playback=setup["playback"])
        if not a.panel:  # from the control window, the sequence (and its spin) waits for Play
            ap.set_defaults(seq=setup["sequence"] or None, spin=setup["spin_deg_s"] if any(setup["spin_deg_s"]) else None)
        a = ap.parse_args()

    if a.model:
        model = mujoco.MjModel.from_xml_path(a.model)
    else:  # always the current hardware spec, never a stale chain.xml
        opts = {**setup_file.build_args(setup), "air": a.air} if setup else dict(air=a.air)
        model = load_model(**opts)
    if a.gravity is not None:
        model.opt.gravity[:] = (0, 0, -a.gravity)
    session = Session(model, mujoco.MjData(model), a.log or None, setup if setup and not a.model else None)
    session.speed, session.playback = a.speed, a.playback
    fly = bool(a.headless and setup and setup["flight"] and not a.model)  # a flight setup flies its parabola
    for axis, rate in zip("xyz", () if fly else a.spin or ()):  # in a flight, the spin comes as the strings let go
        session.spin(rate, axis)
    if fly:
        session.fly(a.seq or "", a.move, a.hold)
    elif a.seq:
        session.play(a.seq, a.move, a.hold)
    try:
        if a.headless:
            assert a.seq or fly, "--headless needs --seq"
            if setup:
                print(f"{os.path.basename(a.setup)}: {setup_file.summary(setup)}")
            while session.fly_t is not None if fly else session.seq_t < session.duration() - 1e-9:
                session.advance()
            secs = session.duration() / session.speed if session.keys is not None else 0
            print(f"ran {secs:g} s" + (f" (longer than a {PARABOLA_S:g} s parabola)" if secs > PARABOLA_S else ""))
            if not fly:  # a flight prints its own summary as it lands
                print(session.summary())
        else:
            run_viewer(session, a.panel)
    finally:
        if session.log:
            session.log.close()
            print(f"log written to {os.path.relpath(session.log.path, HERE)}")
