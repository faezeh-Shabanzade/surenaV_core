#!/usr/bin/env python3
"""Joint-mapping tests: URDF <-> PyBullet consistency for the Surena V.

Run:  python3 test_joint_mapping.py
"""
import os
import sys
import unittest

import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
from pybullet_control_manager import SurenaVControl  # noqa: E402

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "urdf", "surenaV_position.urdf")


class TestJointMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bot = SurenaVControl(gui=False, fixed_base=True)
        cls.urdf_root = ET.parse(URDF).getroot()

    @classmethod
    def tearDownClass(cls):
        cls.bot.close()

    def test_urdf_parses(self):
        self.assertEqual(self.urdf_root.tag, "robot")
        self.assertEqual(self.urdf_root.get("name"), "surenaV")

    def test_no_duplicate_joint_names(self):
        names = [j.get("name") for j in self.urdf_root.findall("joint")]
        self.assertEqual(len(names), len(set(names)),
                         "duplicate joint names in URDF")

    def test_no_duplicate_link_names(self):
        names = [l.get("name") for l in self.urdf_root.findall("link")]
        self.assertEqual(len(names), len(set(names)),
                         "duplicate link names in URDF")

    def test_joint_counts(self):
        joints = self.urdf_root.findall("joint")
        revolute = [j for j in joints if j.get("type") == "revolute"]
        fixed = [j for j in joints if j.get("type") == "fixed"]
        self.assertEqual(len(joints), 62)
        self.assertEqual(len(revolute), 51)
        self.assertEqual(len(fixed), 11)
        self.assertEqual(len(self.bot.joints), 62)
        self.assertEqual(len(self.bot.movable_joints()), 51)

    def test_pybullet_matches_urdf_names(self):
        urdf_names = {j.get("name") for j in self.urdf_root.findall("joint")}
        self.assertEqual(set(self.bot.joints.keys()), urdf_names)

    def test_pybullet_matches_urdf_limits(self):
        for j in self.urdf_root.findall("joint"):
            name = j.get("name")
            if j.get("type") == "fixed":
                continue
            lim = j.find("limit")
            got = self.bot.joint_limits(name)
            self.assertAlmostEqual(got["lower"], float(lim.get("lower")),
                                   places=6, msg=name)
            self.assertAlmostEqual(got["upper"], float(lim.get("upper")),
                                   places=6, msg=name)
            self.assertAlmostEqual(got["max_velocity"],
                                   float(lim.get("velocity")),
                                   places=6, msg=name)
            self.assertAlmostEqual(got["max_effort"], float(lim.get("effort")),
                                   places=6, msg=name)

    def test_transmissions_cover_all_movable_joints(self):
        tr_joints = {t.find("joint").get("name")
                     for t in self.urdf_root.findall("transmission")}
        movable = {j.get("name") for j in self.urdf_root.findall("joint")
                   if j.get("type") != "fixed"}
        self.assertEqual(tr_joints, movable,
                         "transmissions and movable joints disagree")

    def test_transmission_interfaces(self):
        for variant, iface in (("surenaV_position.urdf",
                                "hardware_interface/PositionJointInterface"),
                               ("surenaV_velocity.urdf",
                                "hardware_interface/VelocityJointInterface")):
            path = os.path.join(os.path.dirname(URDF), variant)
            root = ET.parse(path).getroot()
            for t in root.findall("transmission"):
                hw = [e.text.strip()
                      for e in t.iter("hardwareInterface")]
                self.assertEqual(len(hw), 2, t.get("name"))
                self.assertEqual(set(hw), {iface}, t.get("name"))

    def test_fixed_joints_not_in_movable_list(self):
        fixed = {j.get("name") for j in self.urdf_root.findall("joint")
                 if j.get("type") == "fixed"}
        self.assertFalse(fixed & set(self.bot.movable_joints()))

    def test_unknown_joint_raises(self):
        with self.assertRaises(KeyError):
            self.bot.joint_index("not_a_joint")


if __name__ == "__main__":
    unittest.main(verbosity=2)
