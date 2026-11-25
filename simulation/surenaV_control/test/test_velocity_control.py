#!/usr/bin/env python3
"""Velocity-control tests for the Surena V in PyBullet (headless).

Run:  python3 test_velocity_control.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
from pybullet_control_manager import (  # noqa: E402
    SurenaVControl, WrongControlModeError,
)


class TestVelocityControl(unittest.TestCase):
    def setUp(self):
        self.bot = SurenaVControl(gui=False, fixed_base=True)
        self.bot.set_control_mode("velocity")

    def tearDown(self):
        self.bot.close()

    def test_single_joint_velocity(self):
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
        self.bot.run(1.0)
        v = self.bot.get_joint_velocities("r_elbow_pitch_joint")
        self.assertAlmostEqual(v, 0.2, places=1)

    def test_multi_joint_velocity(self):
        self.bot.set_velocity_targets({"r_arm_pitch_joint": 0.1,
                                       "l_arm_pitch_joint": -0.1})
        self.bot.run(1.0)
        got = self.bot.get_joint_velocities(
            ["r_arm_pitch_joint", "l_arm_pitch_joint"])
        self.assertAlmostEqual(got["r_arm_pitch_joint"], 0.1, places=1)
        self.assertAlmostEqual(got["l_arm_pitch_joint"], -0.1, places=1)

    def test_velocity_clamped_to_urdf_limit(self):
        # URDF velocity limit for body joints is 0.5 rad/s. The COMMAND must
        # be clamped; the measured velocity may transiently overshoot a stiff
        # servo (motor force defaults to the URDF effort placeholder), so the
        # measured check allows a 20% transient band.
        self.bot.set_velocity_targets({"l_knee_joint": 50.0})
        self.assertLessEqual(abs(self.bot._velocity_target["l_knee_joint"]),
                             0.5 + 1e-9, "command not clamped")
        self.bot.run(0.5)
        v = self.bot.get_joint_velocities("l_knee_joint")
        self.assertLessEqual(abs(v), 0.5 * 1.2,
                             "measured velocity far beyond the limit")

    def test_acceleration_ramping(self):
        # With a 1 rad/s^2 limit, after 0.1 s the command must be <= 0.1 rad/s.
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 1.0},
                                      max_acceleration=1.0)
        self.bot.run(0.1)
        v = self.bot.get_joint_velocities("r_elbow_pitch_joint")
        self.assertLessEqual(v, 0.1 + 0.02, "acceleration limit violated")

    def test_sequence_of_velocity_targets(self):
        for target in (0.2, -0.2, 0.0):
            self.bot.set_velocity_targets({"head_yaw_joint": target})
            self.bot.run(1.0)
        v = self.bot.get_joint_velocities("head_yaw_joint")
        self.assertLess(abs(v), 0.02, "did not return to rest")

    def test_wrong_mode_rejected(self):
        with self.assertRaises(WrongControlModeError):
            self.bot.set_position_targets({"l_knee_joint": 0.1})

    def test_safe_stop(self):
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 0.3})
        self.bot.run(1.0)
        self.bot.stop_all_motors()
        self.bot.run(1.0)
        v = self.bot.get_joint_velocities("r_elbow_pitch_joint")
        self.assertLess(abs(v), 0.01, "safe stop did not stop the joint")

    def test_watchdog_ramps_to_zero(self):
        self.bot.enable_watchdog(timeout=0.5)
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
        self.bot.run(1.5)   # 1.0 s past the watchdog timeout
        v = self.bot.get_joint_velocities("r_elbow_pitch_joint")
        self.assertLess(abs(v), 0.02, "watchdog failed to ramp to zero")

    def test_emergency_stop_detaches(self):
        self.bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
        self.bot.run(0.5)
        self.bot.emergency_stop()
        # After detaching, the applied motor torque must be zero.
        self.bot.run(0.1)
        fb = self.bot.get_joint_feedback("r_elbow_pitch_joint")
        self.assertAlmostEqual(abs(fb["applied_torque"]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
