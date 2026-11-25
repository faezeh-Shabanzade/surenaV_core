#!/usr/bin/env python3
"""Position-control demo for the Surena V in PyBullet.

Covers: single-joint move, simultaneous multi-joint move, a sequence of
position targets, feedback readout, and a safe stop.  Run headless (default)
or with the GUI:

    python3 pybullet_position_control.py            # headless, prints metrics
    python3 pybullet_position_control.py --gui      # watch it move
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
    bot.report_joints()

    # 1) Single joint to a specified angle -------------------------------
    bot.set_control_mode("position")
    print("\n== 1) single joint: r_elbow_pitch_joint -> 0.5 rad ==")
    bot.move_to_positions({"r_elbow_pitch_joint": 0.5}, duration=2.0)
    fb = bot.get_joint_feedback("r_elbow_pitch_joint")
    print("   reached %.4f rad (target 0.5)" % fb["position"])

    # 2) Multiple joints simultaneously ----------------------------------
    print("\n== 2) multi-joint: wave the right arm ==")
    bot.move_to_positions({
        "r_arm_pitch_joint": -0.3,
        "r_elbow_pitch_joint": -0.9,
        "r_forearm_roll_joint": 0.4,
    }, duration=3.0)
    for name, fb in bot.get_joint_feedback(
            ["r_arm_pitch_joint", "r_elbow_pitch_joint",
             "r_forearm_roll_joint"]).items():
        print("   %-24s %8.4f rad" % (name, fb["position"]))

    # 3) Sequence of position targets ------------------------------------
    print("\n== 3) sequence: head look left -> right -> centre ==")
    sequence = [
        {"head_yaw_joint": 0.6},
        {"head_yaw_joint": -0.6},
        {"head_yaw_joint": 0.0, "head_pitch_joint": 0.3},
        {"head_pitch_joint": 0.0},
    ]
    for i, step_targets in enumerate(sequence, 1):
        bot.move_to_positions(step_targets, duration=1.5)
        done = bot.get_joint_positions(list(step_targets))
        print("   step %d done: %s" % (
            i, {k: round(v, 4) for k, v in done.items()}))

    # 4) Feedback + limits ------------------------------------------------
    print("\n== 4) feedback sample ==")
    fb = bot.get_joint_feedback(["l_knee_joint"])
    print("  ", fb)
    print("   limits:", bot.joint_limits("l_knee_joint"))

    # 5) Safe stop ---------------------------------------------------------
    print("\n== 5) safe stop ==")
    bot.stop_all_motors()
    bot.run(0.5)
    print("   all motors holding their last targets.")
    bot.close()
    print("\nPOSITION DEMO OK")


if __name__ == "__main__":
    main()
