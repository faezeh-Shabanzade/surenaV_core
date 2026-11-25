#!/usr/bin/env python3
"""Static validation of the Surena V control package.

Checks (no ROS or Gazebo required):
  1. All URDFs parse; link/joint sets identical across source and variants;
     variants differ from the source ONLY in transmission hardwareInterface
     tags.
  2. Every controller in every YAML references a real movable URDF joint;
     controller types are the expected ones; fixed joints appear nowhere.
  3. Launch files reference files that exist and joints that exist.
  4. joint_limits.yaml covers every movable joint.

Run:  python3 scripts/validate_urdf_and_configs.py
Exit code 0 = all checks passed.
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_DIR = os.path.join(PKG_DIR, "..")
SOURCE_URDF = os.path.join(SIM_DIR, "surenaV_description",
                           "urdf", "surenaV.urdf")
VARIANTS = ("surenaV_position.urdf", "surenaV_velocity.urdf")

failures = []


def check(condition, message):
    status = "PASS" if condition else "FAIL"
    print("  [%s] %s" % (status, message))
    if not condition:
        failures.append(message)


def hw_stripped(text):
    return re.sub(r"<hardwareInterface>[^<]*</hardwareInterface>", "HW", text)


def main():
    print("== 1. URDF validation ==")
    src_text = open(SOURCE_URDF).read()
    src_root = ET.parse(SOURCE_URDF).getroot()
    src_joints = [j.get("name") for j in src_root.findall("joint")]
    src_links = [l.get("name") for l in src_root.findall("link")]
    movable = [j.get("name") for j in src_root.findall("joint")
               if j.get("type") != "fixed"]
    fixed = [j.get("name") for j in src_root.findall("joint")
             if j.get("type") == "fixed"]
    check(len(src_joints) == len(set(src_joints)), "no duplicate joint names")
    check(len(movable) == 51, "51 movable joints (found %d)" % len(movable))
    check(len(fixed) == 11, "11 fixed joints (found %d)" % len(fixed))

    transmissions = src_root.findall("transmission")
    check(len(transmissions) == 51,
          "51 transmissions (found %d)" % len(transmissions))
    tr_joints = {t.find("joint").get("name") for t in transmissions}
    check(tr_joints == set(movable),
          "transmissions cover exactly the movable joints")

    for variant in VARIANTS:
        path = os.path.join(PKG_DIR, "urdf", variant)
        text = open(path).read()
        root = ET.parse(path).getroot()
        check([j.get("name") for j in root.findall("joint")] == src_joints,
              "%s: joints preserved" % variant)
        check([l.get("name") for l in root.findall("link")] == src_links,
              "%s: links preserved" % variant)
        check(hw_stripped(text) == hw_stripped(src_text),
              "%s: identical to source except hardwareInterface tags" % variant)
        ifaces = {e.text.strip() for e in root.iter("hardwareInterface")}
        expected = ("hardware_interface/PositionJointInterface"
                    if "position" in variant else
                    "hardware_interface/VelocityJointInterface")
        check(ifaces == {expected}, "%s: all interfaces are %s"
              % (variant, expected))

    print("== 2. Controller YAML validation ==")
    yaml_specs = (
        ("config/position_controllers.yaml",
         "position_controllers/JointPositionController", "position"),
        ("config/velocity_controllers.yaml",
         "velocity_controllers/JointVelocityController", "velocity"),
    )
    for rel, expected_type, stem in yaml_specs:
        data = yaml.safe_load(open(os.path.join(PKG_DIR, rel)))["surenaV"]
        controllers = {k: v for k, v in data.items()
                       if isinstance(v, dict) and "type" in v}
        joint_ctrls = {k: v for k, v in controllers.items()
                       if k != "joint_state_controller"}
        check(controllers["joint_state_controller"]["type"] ==
              "joint_state_controller/JointStateController",
              "%s: joint_state_controller present" % rel)
        check(all(v["type"] == expected_type for v in joint_ctrls.values()),
              "%s: all controllers are %s" % (rel, expected_type))
        yaml_joints = {v["joint"] for v in joint_ctrls.values()}
        check(yaml_joints == set(movable),
              "%s: controllers cover exactly the 51 movable joints" % rel)
        check(all(("%s_%s_controller" % (j, stem)) in joint_ctrls
                  for j in movable),
              "%s: controller naming matches <joint>_<stem>_controller" % rel)
        check(not (yaml_joints & set(fixed)),
              "%s: no fixed joints in controller config" % rel)

    traj = yaml.safe_load(
        open(os.path.join(PKG_DIR, "config/trajectory_controllers.yaml")))["surenaV"]
    traj_joints = set()
    for name, cfg in traj.items():
        check(cfg["type"] ==
              "joint_trajectory_controller/JointTrajectoryController",
              "%s: JTC type" % name)
        traj_joints.update(cfg["joints"])
    check(traj_joints == set(movable),
          "trajectory controllers cover exactly the 51 movable joints")

    limits = yaml.safe_load(
        open(os.path.join(PKG_DIR, "config/joint_limits.yaml")))["surenaV"]["joint_limits"]
    check(set(limits.keys()) == set(movable),
          "joint_limits.yaml covers exactly the movable joints")
    urdf_limits_ok = True
    for j in src_root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        entry = limits[j.get("name")]
        urdf_limits_ok &= (
            abs(entry["lower"] - float(lim.get("lower"))) < 1e-9 and
            abs(entry["upper"] - float(lim.get("upper"))) < 1e-9 and
            abs(entry["max_velocity"] - float(lim.get("velocity"))) < 1e-9)
    check(urdf_limits_ok, "joint_limits.yaml values match the URDF <limit> tags")

    print("== 3. Launch file validation ==")
    for rel in ("launch/gazebo_position.launch", "launch/gazebo_velocity.launch",
                "launch/pybullet_position.launch", "launch/pybullet_velocity.launch"):
        path = os.path.join(PKG_DIR, rel)
        try:
            ET.parse(path)
            check(True, "%s: well-formed XML" % rel)
        except ET.ParseError as e:
            check(False, "%s: XML parse error %s" % (rel, e))
    for rel, urdf_ref in (("launch/gazebo_position.launch", "surenaV_position.urdf"),
                          ("launch/gazebo_velocity.launch", "surenaV_velocity.urdf")):
        text = open(os.path.join(PKG_DIR, rel)).read()
        check(urdf_ref in text, "%s: spawns %s" % (rel, urdf_ref))
        yaml_ref = ("position_controllers.yaml" if "position" in rel
                    else "velocity_controllers.yaml")
        check(yaml_ref in text, "%s: loads %s" % (rel, yaml_ref))
    world = os.path.join(SIM_DIR, "surenaV_gazebo", "worlds", "surenaV.world")
    check(os.path.isfile(world), "referenced world file exists")
    bridge = os.path.join(SIM_DIR, "surenaV_gazebo", "src", "gazebo_bridge.cpp")
    check("controller_suffix" in open(bridge).read(),
          "gazebo_bridge.cpp supports the controller_suffix parameter")

    print("== 4. Python scripts compile ==")
    import py_compile
    for script in ("pybullet_control_manager.py", "pybullet_position_control.py",
                   "pybullet_velocity_control.py", "switch_control_mode.py",
                   "pybullet_ros_node.py", "generate_urdf_variants.py"):
        path = os.path.join(PKG_DIR, "scripts", script)
        try:
            py_compile.compile(path, doraise=True)
            check(True, "%s compiles" % script)
        except py_compile.PyCompileError as e:
            check(False, "%s: %s" % (script, e))

    print()
    if failures:
        print("VALIDATION FAILED: %d check(s):" % len(failures))
        for f in failures:
            print("  -", f)
        sys.exit(1)
    print("ALL STATIC VALIDATION CHECKS PASSED")


if __name__ == "__main__":
    main()
