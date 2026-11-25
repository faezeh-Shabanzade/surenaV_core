#!/usr/bin/env python3
"""Position-control tests for the Surena V in PyBullet (headless).

Run:  python3 test_position_control.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
from pybullet_control_manager import (  # noqa: E402
    SurenaVControl, WrongControlModeError,
)


class TestPositionControl(unittest.TestCase):
    def setUp(self):
        self.bot = SurenaVControl(gui=False, fixed_base=True)
        self.bot.set_control_mode("position")

    def tearDown(self):
        self.bot.close()

    def test_single_joint_reaches_target(self):
        self.bot.move_to_positions({"r_elbow_pitch_joint": 0.5}, duration=2.0)
        pos = self.bot.get_joint_positions("r_elbow_pitch_joint")
        self.assertAlmostEqual(pos, 0.5, places=2)

    def test_multi_joint_simultaneous(self):
        targets = {"r_arm_pitch_joint": -0.3, "l_arm_pitch_joint": -0.3,
                   "head_yaw_joint": 0.4}
        self.bot.move_to_positions(targets, duration=3.0)
        got = self.bot.get_joint_positions(list(targets))
        for name, target in targets.items():
            self.assertAlmostEqual(got[name], target, places=2, msg=name)

    def test_sequence_of_targets(self):
        seq = [{"head_yaw_joint": 0.5}, {"head_yaw_joint": -0.5},
               {"head_yaw_joint": 0.0}]
        for step_targets in seq:
            self.bot.move_to_positions(step_targets, duration=1.5)
        self.assertAlmostEqual(
            self.bot.get_joint_positions("head_yaw_joint"), 0.0, places=2)

    def test_position_clamped_to_limits(self):
        # Command far beyond the +-1.57 limit; the joint must stay inside.
        self.bot.set_position_targets({"l_knee_joint": 10.0})
        self.bot.run(1.0)
        pos = self.bot.get_joint_positions("l_knee_joint")
        self.assertLessEqual(pos, self.bot.joint_limits("l_knee_joint")["upper"] + 1e-6)

    def test_wrong_mode_rejected(self):
        with self.assertRaises(WrongControlModeError):
            self.bot.set_velocity_targets({"l_knee_joint": 0.1})

    def test_mode_switch_to_velocity_and_back(self):
        self.bot.move_to_positions({"r_elbow_pitch_joint": 0.3}, duration=1.5)
        self.bot.set_control_mode("velocity")
        self.assertEqual(self.bot.get_control_mode(), "velocity")
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 0.1})
        self.bot.run(0.5)
        self.bot.set_control_mode("position")
        # After switching back, the joint must hold still (target = current).
        p1 = self.bot.get_joint_positions("r_elbow_pitch_joint")
        self.bot.run(0.5)
        p2 = self.bot.get_joint_positions("r_elbow_pitch_joint")
        self.assertLess(abs(p2 - p1), 1e-3)

    def test_safe_stop_holds(self):
        self.bot.move_to_positions({"r_elbow_pitch_joint": 0.4}, duration=1.5)
        self.bot.stop_all_motors()
        self.bot.run(0.5)
        pos = self.bot.get_joint_positions("r_elbow_pitch_joint")
        self.assertAlmostEqual(pos, 0.4, places=2)

    def test_robot_does_not_drift_uncontrolled(self):
        # With all joints at zero targets the robot must stay near zero.
        self.bot.run(2.0)
        worst = max(abs(v) for v in self.bot.get_joint_positions(
            self.bot.movable_joints()).values())
        self.assertLess(worst, 0.05,
                        "uncontrolled drift: worst joint error %.4f rad" % worst)


if __name__ == "__main__":
    unittest.main(verbosity=2)
