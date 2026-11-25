#!/usr/bin/env python3
"""ROS 1 demo: smooth arm trajectories via the joint trajectory controller.

Sends multi-waypoint FollowJointTrajectory goals to the group trajectory
controller (r_arm or l_arm), which interpolates them with its own spline --
the proper ROS way to stream smooth arm motion.

IMPORTANT (ROS 1 limitation): joint_trajectory_controller is built for the
PositionJointInterface only, so this demo works in the POSITION session:

    roslaunch surenaV_control gazebo_position.launch     # or pybullet_position.launch
    rosrun surenaV_control arm_trajectory_demo.py _arm:=r

The trajectory controllers are loaded STOPPED by the session launches; this
node activates the one it needs through controller_manager/switch_controller
and leaves it running afterwards.

Waypoint timing is derived from the URDF velocity limit (0.5 rad/s for body
joints) with margin, so goals are feasible and not rejected.
"""
import argparse

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectory, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ControllerManager, ControllerManagerRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = {
    "r": ["r_arm_pitch_joint", "r_arm_roll_joint", "r_elbow_pitch_joint",
          "r_forearm_roll_joint", "r_forearm_link_joint",
          "r_hand_pitch_joint", "r_hand_roll_joint"],
    "l": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_elbow_pitch_joint",
          "l_forearm_roll_joint", "l_forearm_link_joint",
          "l_hand_pitch_joint", "l_hand_roll_joint"],
}

# Same named poses as the other demos (radians, inside +/-1.57 rad limits).
POSES = {
    "home":          [0.0] * 7,
    "reach_forward": [-0.6, 0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
    "reach_side":    [0.0, 0.7, -0.5, 0.0, 0.0, 0.0, 0.0],
    "wave_up":       [-1.2, 0.0, -0.9, 0.0, 0.0, 0.3, 0.0],
    "wave_out":      [-1.0, 0.0, -0.5, 0.4, 0.0, 0.0, 0.3],
}

PROGRAM = ["home", "reach_forward", "wave_up", "wave_out", "wave_up",
           "wave_out", "home"]

MAX_VELOCITY = 0.5   # rad/s, URDF body limit
VELOCITY_MARGIN = 0.6  # use 60% of the limit for planning


def activate_controller(ns, name):
    """Start a controller that the session launch loaded --stopped."""
    rospy.wait_for_service("%s/controller_manager/switch_controller" % ns,
                           timeout=30.0)
    switch = rospy.ServiceProxy("%s/controller_manager/switch_controller" % ns,
                                ControllerManager)
    req = ControllerManagerRequest()
    req.start_controllers = [name]
    req.stop_controllers = []
    req.strictness = 2  # STRICT: fail loudly if the controller cannot start
    resp = switch(req)
    if not resp.ok:
        raise RuntimeError("switch_controller failed for %s" % name)
    rospy.loginfo("activated %s", name)


def plan_timing(joints, waypoints):
    """Per-segment durations (s) from distance and the URDF velocity limit."""
    durations = []
    for prev, nxt in zip(waypoints, waypoints[1:]):
        max_dist = max(abs(b - a) for a, b in zip(prev, nxt))
        durations.append(max(1.0, max_dist / (MAX_VELOCITY * VELOCITY_MARGIN)))
    return durations


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--arm", choices=["r", "l"], default="r")
    # argparse is fed only extra args ROS did not consume
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("arm_trajectory_demo")
    ns = rospy.get_param("~ns", "/surenaV")
    side = args.arm
    joints = ARM_JOINTS[side]
    controller = "%s_arm_trajectory_controller" % side

    activate_controller(ns, controller)

    # current positions -> first waypoint starts from where the arm is
    state = rospy.wait_for_message("%s/joint_states" % ns, JointState,
                                   timeout=30.0)
    measured = dict(zip(state.name, state.position))
    start = [measured.get(j, 0.0) for j in joints]

    waypoints = [start] + [POSES[name] for name in PROGRAM]
    durations = plan_timing(joints, waypoints)

    traj = JointTrajectory()
    traj.joint_names = joints
    elapsed = 0.0
    for pose, dt in zip(waypoints[1:], durations):
        elapsed += dt
        point = JointTrajectoryPoint()
        point.positions = pose
        point.velocities = [0.0] * len(joints)
        point.time_from_start = rospy.Duration(elapsed)
        traj.points.append(point)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    client = actionlib.SimpleActionClient(
        "%s/%s/follow_joint_trajectory" % (ns, controller),
        FollowJointTrajectory)
    rospy.loginfo("sending %d-waypoint trajectory to %s ...",
                  len(traj.points), controller)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logfatal("trajectory action server not available")
        return 1
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(elapsed + 30.0)):
        rospy.logfatal("trajectory timed out")
        return 1
    result = client.get_result()
    rospy.loginfo("trajectory %s (error code %s)",
                  "FINISHED" if result and result.error_code == 0
                  else "did not finish cleanly",
                  result.error_code if result else "?")
    return 0 if result and result.error_code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
