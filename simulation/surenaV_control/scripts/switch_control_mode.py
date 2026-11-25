#!/usr/bin/env python3
"""Safe control-mode switching demo for the Surena V in PyBullet.

Shows the correct switch sequence and the guards that prevent the two modes
from ever commanding the same joint at the same time:

  1. position mode: move a joint
  2. switch to velocity: stop_all_motors() first, then re-arm in velocity
     mode (velocity commands start at 0 rad/s = no jump)
  3. drive the joint at constant velocity
  4. switch back to position: stop first, then re-arm; position targets are
     re-initialised to the CURRENT joint positions (no jump)
  5. wrong-mode commands raise WrongControlModeError (demonstrated)

    python3 switch_control_mode.py [--gui]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pybullet_control_manager import (  # noqa: E402
    SurenaVControl, WrongControlModeError,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--fixed-base", action="store_true")
    args = parser.parse_args()

    bot = SurenaVControl(gui=args.gui, fixed_base=args.fixed_base)
    joint = "l_elbow_pitch_joint"

    print("== 1) position mode ==")
    bot.set_control_mode("position")
    bot.move_to_positions({joint: 0.4}, duration=2.0)
    print("   %s at %.4f rad" % (joint, bot.get_joint_positions(joint)))

    print("== 2) switch to velocity (stop -> re-arm at 0 rad/s) ==")
    bot.set_control_mode("velocity")
    print("   mode is now:", bot.get_control_mode())

    print("== 3) drive at 0.15 rad/s ==")
    bot.set_velocity_targets({joint: 0.15})
    bot.run(1.0)
    print("   velocity: %.4f rad/s" % bot.get_joint_velocities(joint))

    print("== 4) switch back to position (targets re-init to current pose) ==")
    current = bot.get_joint_positions(joint)
    bot.set_control_mode("position")
    held = bot.get_joint_positions(joint)
    print("   position target re-initialised to %.4f rad (was %.4f)"
          % (held, current))

    print("== 5) wrong-mode guard ==")
    try:
        bot.set_velocity_targets({joint: 0.1})
    except WrongControlModeError as e:
        print("   correctly rejected:", e)

    bot.stop_all_motors()
    bot.close()
    print("\nMODE-SWITCH DEMO OK")


if __name__ == "__main__":
    main()
