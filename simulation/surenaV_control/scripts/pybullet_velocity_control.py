#!/usr/bin/env python3
"""Velocity-control demo for the Surena V in PyBullet.

Covers: single-joint velocity command, simultaneous multi-joint commands, a
sequence of velocity targets, acceleration-limited ramping, the command
watchdog, and a safe stop.  Run headless (default) or with the GUI:

    python3 pybullet_velocity_control.py            # headless, prints metrics
    python3 pybullet_velocity_control.py --gui      # watch it move
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pybullet_control_manager import SurenaVControl  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gui", action="store_true", help="open the PyBullet GUI")
    parser.add_argument("--fixed-base", action="store_true",
                        help="pin the base (pure joint-tracking demo)")
    args = parser.parse_args()

    bot = SurenaVControl(gui=args.gui, fixed_base=args.fixed_base)
    bot.enable_watchdog(timeout=2.0)   # ramp to zero if commands stop arriving

    # 1) Single joint at a specified velocity -----------------------------
    bot.set_control_mode("velocity")
    print("== 1) single joint: r_elbow_pitch_joint at 0.2 rad/s for 1 s ==")
    bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
    bot.run(1.0)
    v = bot.get_joint_velocities("r_elbow_pitch_joint")
    print("   reached %.4f rad/s (target 0.2)" % v)

    # 2) Multiple joints simultaneously -----------------------------------
    print("\n== 2) multi-joint: both shoulders at constant velocity ==")
    bot.set_velocity_targets({
        "r_arm_pitch_joint": 0.15,
        "l_arm_pitch_joint": -0.15,
    })
    bot.run(1.5)
    for name, v in bot.get_joint_velocities(
            ["r_arm_pitch_joint", "l_arm_pitch_joint"]).items():
        print("   %-24s %8.4f rad/s" % (name, v))

    # 3) Sequence of velocity targets (with ramping) ----------------------
    print("\n== 3) sequence: knee velocity ramp +0.3 -> -0.3 -> 0 ==")
    for target in (0.3, -0.3, 0.0):
        bot.set_velocity_targets({"l_knee_joint": target})
        bot.run(1.0)
        v = bot.get_joint_velocities("l_knee_joint")
        print("   target %+.1f -> measured %+.4f rad/s" % (target, v))

    # 4) Watchdog: stop sending commands, joint must ramp to rest ---------
    print("\n== 4) watchdog: no commands for >2 s -> automatic ramp to zero ==")
    bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
    bot.run(2.5)   # past the 2 s watchdog timeout; no refresh
    v = bot.get_joint_velocities("r_elbow_pitch_joint")
    print("   velocity after watchdog: %.4f rad/s (should be ~0)" % v)

    # 5) Safe stop ---------------------------------------------------------
    print("\n== 5) safe stop ==")
    bot.stop_all_motors()
    bot.run(0.5)
    print("   all velocity targets zeroed; motors hold against gravity.")
    bot.close()
    print("\nVELOCITY DEMO OK")


if __name__ == "__main__":
    main()
