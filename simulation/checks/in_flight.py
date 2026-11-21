"""Sections 7 and 15: the flight options (IMU, weighed modules, air drag, the box) and a parabola in the flight box."""
import contextlib
import io
import math
import os

import numpy as np

import setup_file
from flight import Parabola as Play_parabola
from make_box import held_height
from make_chain import PRESETS
from sequence import LOG_DT
from session import Session
from spec import FLIGHT
from checks.common import check, load, HERE


def check_flight():
    print("7. flight options: IMU output, weighed modules, air drag, the flight box")
    m, d = load()
    s = Session(m, d)
    s.spin(10, "x")
    while d.time < 2 - 1e-9:
        s.advance()
    rate = math.degrees(np.linalg.norm(d.sensor("imu_gyro").data))
    check(abs(rate - 10) < 0.05 and abs(s.imu_turn() - 20) < 0.2,
          f"IMU: spinning at 10 deg/s for 2 s, its gyro reads {rate:.2f} deg/s and it has turned {s.imu_turn():.2f} deg")
    PRESETS["weighed"] = {**PRESETS["original14"], "module_masses": [0.25] * 12}  # borrowed for this check only
    try:
        m, _ = load("weighed")
    finally:
        del PRESETS["weighed"]
    check(abs(sum(m.body_mass) - 3.0) < 1e-9, f"12 weighed modules of 250 g make a {sum(m.body_mass):.3f} kg chain")
    after = []
    for air in (False, True):
        m, d = load(air=air)
        s = Session(m, d)
        s.spin(20, "z")
        while d.time < 10 - 1e-9:
            s.advance()
        after.append(s.spin_rate())
    check(abs(after[0] - 20) < 0.01 and 15 < after[1] < 19.8,
          f"air drag: spinning at 20 deg/s, after 10 s it's {after[1]:.2f} deg/s with cabin air, {after[0]:.2f} without")
    m, d = load(flight=dict(box_m=[2.5, 1.3, 1.9], strings=[]))
    s, hit = Session(m, d), None
    start = d.ncon
    m.opt.gravity[:] = (0, 0, -0.0981)
    while hit is None and d.time < 8:
        s.advance()
        hit = d.time if d.ncon else None
    fall = held_height([2.5, 1.3, 1.9]) - m.geom("m1_sphere").size[0] - FLIGHT["net_height_mm"] / 1000
    want = math.sqrt(2 * fall / 0.0981)
    check(not start and hit is not None and abs(hit - want) < 0.2,
          f"flight box: nothing touches at the start in the middle; with 0.01 g left over, the chain reaches the net after "
          f"{f'{hit:.2f} s' if hit else 'never'} (falling {fall:.2f} m: {want:.2f} s)")


def check_flight_parabola():
    print("15. a parabola in the flight box (8.5 cm modules, setups/1a_turn.toml, 1.8 g phases shortened to 7 s)")
    setup = setup_file.load(os.path.join(setup_file.SETUP_DIR, "1a_turn.toml"))
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    start = s.box_com()
    s.fly(hyper_s=7, jitter=False)
    held, pull, top, after, plane = 0.0, 0.0, 0.0, None, None
    with contextlib.redirect_stdout(io.StringIO()):
        while s.fly_t is not None:
            phase, g, T, theta = s.parabola.at(s.fly_t)
            s.advance()
            if T < 0:  # where the chain is in the box, while the plane pitches up
                here = s.box_com()
                held, pull, top = max(held, np.linalg.norm(here - start)), max(pull, s.tension()), max(top, theta)
            if plane is None and s.released is not None:
                plane = math.degrees(s.parabola.pitch(s.fly_t) - s.parabola.pitch(s.fly_t - LOG_DT)) / LOG_DT
                carried = np.cross((math.radians(plane), 0, 0), d.subtree_com[1] - d.xpos[s.cabin])  # the box's speed there
            if after is None and T >= s.release_s + 2:  # 2 s after letting go, nothing pushing it
                after = np.linalg.norm(d.sensor("com_vel").data - carried)
    R, c = s.box_frame(), d.xpos[s.cabin]  # the lowest shell's bottom, above the box floor (in the box's frame)
    low = min((R.T @ (d.geom_xpos[g] - c))[2] - m.geom_size[g][0] for g in s.shells) + setup["flight"]["box_m"][2] / 2
    check(held < 0.005 and pull > 1, f"held by {len(s.strings)} strings through 1 g and 1.8 g as the plane pitches up to "
          f"{math.degrees(top):.0f} deg: the chain stays within {held * 1000:.1f} mm of its place in the box, strings pull up to "
          f"{pull:.1f} N")
    check(after < 0.001 and abs(s.release_spin - plane) < 0.3,
          f"let go at T+{s.released:.1f} s (no jitter): it leaves turning with the plane ({s.release_spin:+.2f} deg/s, the "
          f"plane {plane:+.2f}) and moving with the box where it was held: 2 s later {after * 1000:.2f} mm/s off that")
    check(s.landing is not None and s.dip < FLIGHT["net_height_mm"] / 1000 and low > 0,
          f"the pull-out drops it onto the net at {s.landing:.2f} m/s; it sinks {s.dip * 1000:.0f} mm of the net's "
          f"{FLIGHT['net_height_mm']} mm and never reaches the box floor")
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    s.fly(setup["sequence"], setup["move_s"], setup["hold_s"], hyper_s=7)
    turned = None
    with contextlib.redirect_stdout(io.StringIO()):
        while s.fly_t is not None:
            s.advance()
            if turned is None and s.keys is not None and s.seq_t >= s.duration():
                turned = s.turned  # the turn as the sequence ends, before the pull-out
    check(turned is not None and s.landing is not None, f"with the jitter and the turn sequence, it flies and lands")
    print(f"  info  ... and turns {turned:+.1f} deg about the box's vertical by the sequence's end (-30.3 in perfect zero g, "
          f"let go at rest); {s.flight_result()}")
    setup = setup_file.load(os.path.join(setup_file.SETUP_DIR, "4a_walk.toml"))
    m, d = load(**setup_file.build_args(setup))
    s = Session(m, d, None, setup)
    s.fly(setup["sequence"], setup["move_s"], setup["hold_s"], hyper_s=7, jitter=False)
    with contextlib.redirect_stdout(io.StringIO()):
        while s.keys is None or s.seq_t < s.duration() - 1e-9:
            s.advance()
    walked = -(s.box_com() - s.com0_box)[1]
    check(walked > 0.6 and all(e.pops == 0 for e in s.ends.values()), f"setups/4a_walk: standing on the deck through 1.8 g as "
          f"the box pitches, then walking in zero g on the turning deck: {walked * 1000:.0f} mm, no cup popped")
    p = Play_parabola(7, False, 1, 0.38)
    check(abs(-p.at(p.t0 + 5)[1][2] - 0.38) < 1e-9 and abs(math.degrees(p.top) - math.degrees(math.atan(0.62))) < 1e-9,
          f"a Mars parabola holds 0.38 g and pitches over less: nose up {math.degrees(p.top):.1f} deg (zero g: "
          f"{FLIGHT['pitch_deg']} deg)")
    m, d = load("feetech85")
    s = Session(m, d)
    s.spin(-0.5, "x")
    s.play(os.path.join(os.path.dirname(HERE), "sequences", "reorient_loop.csv"), 1.0, 0.25)
    while s.seq_t < s.duration() - 1e-9:
        s.advance()
    check(abs(s.turned + 20.9) < 0.5, f"why: a free chain let go spinning 0.5 deg/s about the across axis turns "
          f"{s.turned:+.1f} deg, not -30.3 (folding turns that spin into roll about its length), and its IMU turns "
          f"{s.imu_turn():.0f} deg")
