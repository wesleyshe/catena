"""Check the generated model and render PNGs into renders/.

  .venv/bin/python verify.py

1. meshes/ were exported from the current CAD (../hardware/cad), and every preset (with and without --slip-ring) compiles
2. each joint turns about the intended axis: anchors/axes, a 30 deg test turn, and the arm mesh
   (yoke discs centred on the pitch axis, hub centred on the yaw axis, hub top at +88 mm)
3. zero gravity + smooth random joint motion: COM stays still, total angular momentum stays ~0
4. no contacts with the chain straight, and none during ../sequences/fold_sequence.csv
5. gravity on: the chain drops onto the floor and settles there
6. spin: the Spin command (one axis at a time), the spin-up when folded (skater effect), Stop spin, the tip-turn readout
7. flight options: the IMU output, weighed modules, air drag and the flight box
8. offscreen renders: one module, the straight 12-module chain, the L and zig-zag folds, the dropped chain
9. end effectors: every type at each end compiles, the modules are unchanged, the m1_yaw twist axis, zero-g momentum
10. anchors: placement, the clamp holds, grab on touch, let go keeps the momentum, the grip column
11. suction cups: hold below the pull limit, pop above it, grab on touch, never stick to the chain
12. thrusters: a = F/m, opposite nozzles cancel, a side nozzle turns the chain but can't roll it
13. wrapped around a pole: the start pose touches all round, and the wrap-and-inch climb gets up it
14. the countdown before a take: the spin and play wait for it, Stop calls it off, Play again starts it afresh
15. a parabola in the flight box (8.5 cm modules): the strings hold it through 1.8 g, let go cleanly at zero g, the turn
    survives the leftover g, the pull-out lands it on the net
16. setup files: mistakes are refused, Save as then Load gives the same setup, every setup, demo button and sequence is
    in order; the control window imports only the stdlib

The checks are in checks/, one file per subject; this file runs them in order. A number (or several) runs just those
sections: .venv/bin/python verify.py 15
"""
import sys

from checks import common, controls, end_effectors, in_flight, model, physics, renders

SECTIONS = [model.check_compile, model.check_axes, physics.check_momentum, model.check_contacts, physics.check_gravity, physics.check_spin, in_flight.check_flight, renders.renders, end_effectors.check_end_effectors, end_effectors.check_anchors, end_effectors.check_suction, end_effectors.check_thrusters, physics.check_pole_climb, controls.check_countdown, in_flight.check_flight_parabola,
            controls.check_setup_files]  # in the order they print, 1 to 16

if __name__ == "__main__":
    picked = [int(a) for a in sys.argv[1:]] or range(1, len(SECTIONS) + 1)
    for k in picked:
        SECTIONS[k - 1]()
    print("\nALL CHECKS PASSED" if not common.failures else f"\n{len(common.failures)} CHECK(S) FAILED")
    raise SystemExit(bool(common.failures))
