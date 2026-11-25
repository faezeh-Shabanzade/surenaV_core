#!/usr/bin/env python3
"""ROS 1 wrapper around the PyBullet Surena V control manager.

Mirrors the Gazebo Classic controller topics so the same high-level pipeline
(robot_manager / gazebo_bridge-style nodes, rqt tools) runs unchanged against
PyBullet:

  subscribes   /surenaV/<joint>_<mode>_controller/command   std_msgs/Float64
               (one topic per movable joint, <mode> = position|velocity)
  publishes    /surenaV/joint_states                        sensor_msgs/JointState
               (positions, velocities, applied torques)

The control mode is selected at launch with the ~mode parameter; switching
modes means restarting the node with the other mode (see
launch/pybullet_position.launch vs launch/pybullet_velocity.launch), mirroring
the Gazebo Classic workflow.

The node never imports gazebo code; PyBullet's control mode is set explicitly
here (PyBullet does not read URDF transmission tags).

Parameters:
    ~mode            "position" | "velocity"   (default: position)
    ~urdf            absolute URDF path (default: the package's position variant)
    ~fixed_base      bool (default false)
    ~gui             bool (default false)
    ~rate            joint-state publish rate in Hz (default 100)
    ~watchdog        seconds without a command before a joint ramps to rest
                     (velocity mode only; 0 disables, default 0.5)
"""
import os
import sys

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from pybullet_control_manager import (  # noqa: E402
    SurenaVControl, WrongControlModeError,
)


class PyBulletRosNode:
    def __init__(self):
        mode = rospy.get_param("~mode", "position")
        if mode not in (SurenaVControl.POSITION, SurenaVControl.VELOCITY):
            rospy.logfatal("~mode must be 'position' or 'velocity'")
            raise SystemExit(1)
        urdf = rospy.get_param(
            "~urdf",
            os.path.join(_SCRIPTS_DIR, "..", "urdf",
                         "surenaV_%s.urdf" % mode))
        self.bot = SurenaVControl(
            urdf_path=urdf,
            gui=bool(rospy.get_param("~gui", False)),
            fixed_base=bool(rospy.get_param("~fixed_base", False)),
            time_step=1.0 / float(rospy.get_param("~sim_rate_hz", 500)),
        )
        self.bot.set_control_mode(mode)
        watchdog = float(rospy.get_param("~watchdog", 0.5))
        if watchdog > 0 and mode == SurenaVControl.VELOCITY:
            self.bot.enable_watchdog(watchdog)

        self.mode = mode
        self.suffix = "%s_joint_%s_controller" % ("%s", mode)
        self.pubs = {}
        self.subs = {}
        self.latest = {}   # joint name -> last commanded value
        ns = rospy.get_param("~namespace", "/surenaV")
        for name in self.bot.movable_joints():
            topic = "%s/%s" % (ns, self.suffix % name)
            self.pubs[name] = rospy.Publisher(topic, Float64, queue_size=10)
            self.subs[name] = rospy.Subscriber(
                topic, Float64, self._command_cb, callback_args=name,
                queue_size=10)
        self.state_pub = rospy.Publisher("%s/joint_states" % ns, JointState,
                                         queue_size=100)
        rospy.loginfo("PyBullet Surena V up: mode=%s, %d joints, URDF=%s",
                      mode, len(self.bot.movable_joints()), urdf)

    def _command_cb(self, msg, joint_name):
        self.latest[joint_name] = msg.data

    def _publish_states(self):
        names = self.bot.movable_joints()
        fb = self.bot.get_joint_feedback(names)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = [fb[n]["position"] for n in names]
        msg.velocity = [fb[n]["velocity"] for n in names]
        msg.effort = [fb[n]["applied_torque"] for n in names]
        self.state_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(float(rospy.get_param("~rate", 100)))
        while not rospy.is_shutdown():
            if self.latest:
                try:
                    if self.mode == SurenaVControl.POSITION:
                        self.bot.set_position_targets(dict(self.latest))
                    else:
                        self.bot.set_velocity_targets(dict(self.latest))
                except WrongControlModeError as e:
                    rospy.logerr_throttle(5.0, str(e))
            self.bot.step()
            self._publish_states()
            rate.sleep()
        self.bot.close()


if __name__ == "__main__":
    rospy.init_node("pybullet_control_node")
    PyBulletRosNode().spin()
