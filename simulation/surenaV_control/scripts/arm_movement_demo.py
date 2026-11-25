#!/usr/bin/env python3
"""ROS 1 demo node: move the Surena V arm with position or velocity control.

Publishes to the standard per-joint controller topics, so the SAME node runs
against either backend without changes:

  * Gazebo Classic:  roslaunch surenaV_control gazebo_position.launch
                     roslaunch surenaV_control gazebo_velocity.launch
  * PyBullet bridge: roslaunch surenaV_control pybullet_position.launch
                     roslaunch surenaV_control pybullet_velocity.launch

Program (mirrors scripts/demo_arm_movement.py):
  position mode   named poses (home, reach forward, reach side, wave) sent
                  as rate-limited interpolation streams to
                  /surenaV/<joint>_position_controller/command
  velocity mode   sinusoidal / constant velocity sweeps sent to
                  /surenaV/<joint>_velocity_controller/command, with a
                  software slew limit (soft start / soft stop)

No second PID loop runs here: commands go straight to the controllers, whose
tracking is handled by the simulator servos (or, on hardware, by the EPOS4
actuators' internal position/velocity modes).

Run standalone (after starting one of the launches above):

    rosrun surenaV_control arm_movement_demo.py _mode:=position _arm:=r
    rosrun surenaV_control arm_movement_demo.py _mode:=velocity _arm:=both

or use the bundled launch files (arm_demo_position.launch etc.), which start
the simulator session and this node together.
"""
import math

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# The 7 arm joints per side (same list the r_arm/l_arm trajectory
# controllers use).
ARM_JOINTS = {
    "r": ["r_arm_pitch_joint", "r_arm_roll_joint", "r_elbow_pitch_joint",
          "r_forearm_roll_joint", "r_forearm_link_joint",
          "r_hand_pitch_joint", "r_hand_roll_joint"],
    "l": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_elbow_pitch_joint",
          "l_forearm_roll_joint", "l_forearm_link_joint",
          "l_hand_pitch_joint", "l_hand_roll_joint"],
}

# Joint-name fragments that change sign when mirrored onto the left arm.
_MIRROR_NEGATE = ("roll",)

# Named poses, keyed by joint-name fragment (radians). All inside the URDF's
# +/-1.57 rad limits.
POSES = {
    "home":          {"arm_pitch": 0.0, "arm_roll": 0.0,
                      "elbow_pitch": 0.0, "forearm_roll": 0.0,
                      "forearm_link": 0.0, "hand_pitch": 0.0,
                      "hand_roll": 0.0},
    "reach_forward": {"arm_pitch": -0.6, "elbow_pitch": -0.3},
    "reach_side":    {"arm_roll": 0.7, "elbow_pitch": -0.5},
    "wave_up":       {"arm_pitch": -1.2, "elbow_pitch": -0.9,
                      "hand_pitch": 0.3},
    "wave_out":      {"arm_pitch": -1.0, "elbow_pitch": -0.5,
                      "forearm_roll": 0.4, "hand_roll": 0.3},
}

POSITION_PROGRAM = [
    ("home", 0.5), ("reach_forward", 1.0), ("reach_side", 1.0),
    ("home", 0.5), ("wave_up", 1.0), ("wave_out", 0.8), ("wave_up", 0.8),
    ("wave_out", 0.8), ("home", 1.0),
]

# (label, joint fragment, amplitude rad/s, frequency Hz or None, duration s)
VEL_SWEEPS = [
    ("shoulder-pitch sine sweep", "arm_pitch", 0.20, 0.25, 8.0),
    ("elbow constant sweep", "elbow_pitch", 0.15, None, 6.0),
    ("forearm-roll sine sweep", "forearm_roll", 0.15, 0.3, 6.0),
]

# URDF limits (body joints): 0.5 rad/s velocity. Used for interpolation
# timing and the velocity slew limit. Acceleration is a demo choice.
MAX_VELOCITY = 0.5          # rad/s (URDF body limit)
MAX_ACCELERATION = 1.0      # rad/s^2 (demo slew limit, not a spec)
TOLERANCE = 0.05            # rad


def build_pose(side, fragments):
    """Expand {fragment: angle} into {full joint name: angle}."""
    out = {}
    for frag, angle in fragments.items():
        for candidate in ARM_JOINTS[side]:
            core = "_".join(candidate.split("_")[1:-1])
            if core == frag:
                out[candidate] = angle
                break
        else:
            raise KeyError("no %s-arm joint matches fragment '%s'"
                           % (side, frag))
    return out


def mirror(side, pose):
    """Mirror a right-arm pose onto the left arm (negate roll-type joints)."""
    return {j: (-a if any(f in j for f in _MIRROR_NEGATE) else a)
            for j, a in pose.items()}


class ArmMovementDemo(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "position")
        arm = rospy.get_param("~arm", "r")
        if arm not in ("r", "l", "both"):
            rospy.logfatal("~arm must be r, l or both")
            raise rospy.ROSInitException("bad ~arm param")
        self.sides = ["r", "l"] if arm == "both" else [arm]
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        ns = rospy.get_param("~ns", "/surenaV")
        suffix = "position" if self.mode == "position" else "velocity"

        self.pubs = {}
        for side in self.sides:
            for joint in ARM_JOINTS[side]:
                topic = "%s/%s_%s_controller/command" % (ns, joint, suffix)
                self.pubs[joint] = rospy.Publisher(topic, Float64,
                                                   queue_size=1)
        self.current = {}     # joint -> last measured position (rad)
        self.have_state = False
        rospy.Subscriber("%s/joint_states" % ns, JointState,
                         self._joint_state_cb, queue_size=1)

        # Wait for the robot state before moving (the Gazebo session starts
        # paused so controllers can latch zero setpoints first).
        rospy.loginfo("waiting for %s/joint_states ...", ns)
        if not rospy.wait_for_message("%s/joint_states" % ns, JointState,
                                      timeout=30.0):
            raise rospy.ROSInitException("no joint states -- is the "
                                         "simulator session running?")
        rospy.sleep(1.0)  # let controllers finish latching
        rospy.loginfo("arm movement demo: mode=%s arm=%s speed=%.2f "
                      "(%d command topics)", self.mode, arm, self.speed,
                      len(self.pubs))

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos
        self.have_state = True

    # ---------------------------------------------------------------- #
    def run(self):
        if self.mode == "position":
            self.run_position()
        else:
            self.run_velocity()
        rospy.loginfo("demo finished")

    def run_position(self):
        rate = rospy.Rate(self.rate_hz)
        for pose_name, hold in POSITION_PROGRAM:
            if rospy.is_shutdown():
                return
            targets = {}
            for side in self.sides:
                pose = build_pose(side, POSES[pose_name])
                targets.update(pose if side == "r" else mirror(side, pose))
            start = {j: self.current.get(j, 0.0) for j in targets}
            max_dist = max(abs(targets[j] - start[j]) for j in targets)
            duration = max(1.0, max_dist / (0.25 * self.speed))
            rospy.loginfo("pose %-14s (%.1f s)", pose_name, duration)
            steps = max(1, int(duration * self.rate_hz))
            for i in range(1, steps + 1):
                if rospy.is_shutdown():
                    return
                a = 0.5 - 0.5 * math.cos(math.pi * i / steps)  # smooth ease
                for joint, target in targets.items():
                    self.pubs[joint].publish(start[j] + a * (target - start[j]))
                rate.sleep()
            rospy.sleep(hold / self.speed)
            worst = max(abs(self.current.get(j, target) - target)
                        for j, target in targets.items())
            rospy.loginfo("  max |error| = %.4f rad %s",
                          worst, "(OK)" if worst < TOLERANCE else "(DRIFT)")

    def run_velocity(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        for label, frag, amplitude, freq, duration in VEL_SWEEPS:
            if rospy.is_shutdown():
                return
            rospy.loginfo("%s  (%.2f rad/s, %.1f s)", label,
                          amplitude * self.speed, duration)
            joints = []
            for side in self.sides:
                joints.extend(build_pose(side, {frag: 0.0}))
            command = {j: 0.0 for j in joints}
            steps = int(duration * self.speed / dt)
            for i in range(steps):
                if rospy.is_shutdown():
                    # soft stop: slew commands to zero
                    break
                t = i * dt
                if freq is None:
                    v = amplitude * self.speed if (t // 2.0) % 2 == 0 \
                        else -amplitude * self.speed
                else:
                    v = amplitude * self.speed * math.sin(
                        2.0 * math.pi * freq * t)
                for j in joints:
                    sign = -1.0 if ("l_" in j and "roll" in frag) else 1.0
                    target = sign * v
                    # slew limit toward the target (soft start / reversal)
                    step = MAX_ACCELERATION * self.speed * dt
                    command[j] += max(-step, min(step, target - command[j]))
                    self.pubs[j].publish(command[j])
                rate.sleep()
            # soft stop
            while any(abs(c) > 1e-3 for c in command.values()):
                if rospy.is_shutdown():
                    return
                for j in joints:
                    step = MAX_ACCELERATION * self.speed * dt
                    command[j] += max(-step, min(step, -command[j]))
                    self.pubs[j].publish(command[j])
                rate.sleep()
            rospy.sleep(0.5 / self.speed)


def main():
    rospy.init_node("arm_movement_demo")
    ArmMovementDemo().run()


if __name__ == "__main__":
    main()
