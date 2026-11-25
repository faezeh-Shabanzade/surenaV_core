#!/usr/bin/env python3
"""Quick smoke test for the SurenaVControl manager (headless)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
from pybullet_control_manager import SurenaVControl

bot = SurenaVControl(gui=False, fixed_base=True)
print("movable joints:", len(bot.movable_joints()))

bot.set_control_mode("position")
bot.move_to_positions({"r_elbow_pitch_joint": 0.5}, duration=1.0)
fb = bot.get_joint_feedback("r_elbow_pitch_joint")["r_elbow_pitch_joint"]
print("after move: pos=%.4f (target 0.5)" % fb["position"])

bot.set_control_mode("velocity")
bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
bot.run(1.0)
v = bot.get_joint_velocities("r_elbow_pitch_joint")["r_elbow_pitch_joint"]
print("vel after 1s: %.4f (target 0.2)" % v)

bot.stop_all_motors()
bot.close()
print("SMOKE TEST OK")
