#!/usr/bin/env python3
"""Standalone PyBullet control interface for the Surena V humanoid.

This module is deliberately ROS-free: it talks to PyBullet directly and can be
used from plain Python scripts, the ROS wrapper node, or the tests.

Control model
-------------
PyBullet does NOT read the URDF <transmission> hardware-interface tags; the
control mode is set explicitly per joint through
``p.setJointMotorControl2``:

  position mode -> p.POSITION_CONTROL  (ideal position servo; gains and
                    max motor force configurable, velocity capped)
  velocity mode -> p.VELOCITY_CONTROL  (ideal velocity servo; target velocity
                    capped, motor force capped, acceleration-limited ramping)

Exactly one mode is active at a time (``set_control_mode``); commanding in the
wrong mode raises :class:`WrongControlModeError`, so the two modes can never
command the same joint simultaneously.

Safety
------
* Position targets are clamped to the URDF joint limits.
* Velocity targets are clamped to the URDF velocity limits.
* Motor force defaults to the URDF effort limit (NOTE: effort=1000 N.m on body
  joints is the SolidWorks export placeholder, not a verified actuator spec;
  override with ``set_motor_force`` or ``joint_limits.yaml`` values when real
  limits are known).
* Velocity commands are ramped at a configurable acceleration limit.
* ``stop_all_motors()`` brings every joint to a controlled stop;
  ``emergency_stop()`` detaches every motor immediately.

Example
-------
    from pybullet_control_manager import SurenaVControl

    bot = SurenaVControl(gui=True)
    bot.set_control_mode("position")
    bot.move_to_positions({"r_elbow_pitch_joint": 0.5}, duration=2.0)
    print(bot.get_joint_feedback(["r_elbow_pitch_joint"]))
    bot.stop_all_motors()
    bot.close()
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import pybullet as p

__all__ = [
    "SurenaVControl",
    "WrongControlModeError",
    "UnknownJointError",
    "DEFAULT_URDF",
]

# Default: the generated position variant of the final hands URDF. Any of the
# three (source / position / velocity) loads identically in PyBullet -- the
# transmission tags are ignored by PyBullet.
DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "surenaV_position.urdf"
)

# Package whose meshes the URDF references (package://surenaV_description/meshes);
# used so the package:// mesh URIs resolve regardless of where the URDF sits. Can
# be overridden with the package_root argument of SurenaVControl.
_DEFAULT_PACKAGE_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."
)

_MESH_PACKAGE = "surenaV_description"


class WrongControlModeError(RuntimeError):
    """Raised when a command does not match the active control mode."""


class UnknownJointError(KeyError):
    """Raised when a joint name is not part of the model."""


class SurenaVControl:
    """Position/velocity control of the Surena V URDF in PyBullet."""

    POSITION = "position"
    VELOCITY = "velocity"
    _MODES = (POSITION, VELOCITY)

    def __init__(
        self,
        urdf_path: str = DEFAULT_URDF,
        gui: bool = False,
        fixed_base: bool = False,
        enable_self_collision: bool = False,
        time_step: float = 1.0 / 500.0,
        gravity: float = -9.81,
        start_pose_z: float = 0.0,
        package_root: str = None,
    ):
        """Load the robot and set up the simulation.

        Args:
            urdf_path: URDF to load (defaults to the position variant of the
                final Surena V hands model).
            gui: True -> PyBullet GUI (SHARED_MEMORY GUI window), False -> headless.
            fixed_base: pin base_link to the world (useful for pure joint
                tracking tests); False = free-floating standing robot.
            enable_self_collision: enable URDF_USE_SELF_COLLISION. Default
                False to match the Gazebo setup this model was validated in
                (the URDF declares no <selfCollide> tags); enabling it adds
                contacts (e.g. arm-vs-torso) that Gazebo does not simulate
                and that will fight the joint servos.
            time_step: fixed simulation step in seconds.
            gravity: gravitational acceleration (z).
            start_pose_z: initial base height offset (m).
            package_root: directory that CONTAINS the surenaV_description
                package (i.e. .../surenaV_description/meshes/*.STL must
                exist below it). Defaults to the parent of the package this
                script lives in; override when the meshes live elsewhere.
        """
        urdf_path = os.path.abspath(urdf_path)
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError("URDF not found: %s" % urdf_path)

        self.time_step = time_step
        self.gravity = gravity
        self._mode = None
        self._last_command_time = None
        self._velocity_target = {}       # joint name -> clamped target velocity
        self._velocity_command = {}      # joint name -> ramped command (per step)
        self._velocity_accel = {}        # joint name -> acceleration limit
        self._position_target = {}       # joint name -> last position target
        self._motor_force = {}           # joint name -> motor force cap
        self._watchdog_timeout = None    # seconds; None disables
        self._sim_time = 0.0             # accumulated simulated time (s)
        self._urdf_path = urdf_path

        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("pybullet.connect failed")

        p.resetSimulation(self.client)
        p.setGravity(0, 0, gravity, physicsClientId=self.client)
        p.setTimeStep(time_step, physicsClientId=self.client)

        # Ground plane from PyBullet's bundled data, then repoint the search
        # path at the catkin package root so the robot's
        # package://surenaV_description/meshes/... URIs resolve.
        import pybullet as _pb
        _pb_data = os.path.join(os.path.dirname(_pb.__file__), "pybullet_data")
        p.setAdditionalSearchPath(_pb_data, physicsClientId=self.client)
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        package_root = self._find_package_root(urdf_path, package_root)
        p.setAdditionalSearchPath(package_root, physicsClientId=self.client)

        flags = p.URDF_USE_SELF_COLLISION if enable_self_collision else 0
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0.0, 0.0, start_pose_z],
            useFixedBase=fixed_base,
            flags=flags,
            physicsClientId=self.client,
        )

        self._build_joint_map()
        # Conservative default: hold the zero (home) pose with every motor so
        # the robot never starts uncontrolled.
        self._hold_zero_pose()

    # ------------------------------------------------------------------ #
    # Joint mapping / info
    # ------------------------------------------------------------------ #
    def _build_joint_map(self):
        """Map URDF joint names -> PyBullet joint indices, types and limits."""
        self.joints = {}   # name -> dict(index, type, limits, ...)
        self._index_to_name = {}
        for idx in range(p.getNumJoints(self.robot_id, physicsClientId=self.client)):
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client)
            name = info[1].decode("utf-8")
            jtype = info[2]
            lower, upper = info[8], info[9]
            max_effort, max_velocity = info[10], info[11]
            self.joints[name] = {
                "index": idx,
                "type": jtype,
                "type_name": {
                    p.JOINT_REVOLUTE: "revolute",
                    p.JOINT_FIXED: "fixed",
                    p.JOINT_PRISMATIC: "prismatic",
                    p.JOINT_PLANAR: "planar",
                    p.JOINT_SPHERICAL: "spherical",
                }.get(jtype, "other(%d)" % jtype),
                "lower": lower,
                "upper": upper,
                "max_effort": max_effort,
                "max_velocity": max_velocity,
                "child_link": info[12].decode("utf-8"),
                "parent_link": None,  # filled from the URDF below
            }
            self._index_to_name[idx] = name
            self._motor_force[name] = max_effort

        # Parent links from the URDF (PyBullet joint info does not expose them)
        try:
            root = ET.parse(self._urdf_path).getroot()
            for j in root.findall("joint"):
                name = j.get("name")
                if name in self.joints:
                    self.joints[name]["parent_link"] = j.find("parent").get("link")
        except (OSError, ET.ParseError):
            pass  # cosmetic only

        self.movable_joint_names = [
            n for n, j in self.joints.items() if j["type"] != p.JOINT_FIXED
        ]

    def _urdf_source_path(self):
        return self._urdf_path

    @property
    def urdf_path(self):
        return self._urdf_path

    def _hold_zero_pose(self):
        """Engage every movable joint's motor holding its current position."""
        for name in self.movable_joint_names:
            idx = self.joints[name]["index"]
            pos, _vel, _rf, _tor = p.getJointState(
                self.robot_id, idx, physicsClientId=self.client)[:4]
            p.setJointMotorControl2(
                self.robot_id, idx, p.POSITION_CONTROL,
                targetPosition=pos, force=self._motor_force[name],
                physicsClientId=self.client)
            self._position_target[name] = pos
            self._velocity_command[name] = 0.0

    def joint_names(self):
        """All joint names (fixed included), in URDF/PyBullet order."""
        return list(self.joints.keys())

    def movable_joints(self):
        """All movable (actuated) joint names."""
        return list(self.movable_joint_names)

    def joint_index(self, name):
        self._require_joint(name)
        return self.joints[name]["index"]

    def joint_limits(self, name):
        """Position/velocity/effort limits for a joint (from the URDF)."""
        self._require_joint(name)
        j = self.joints[name]
        return {
            "lower": j["lower"],
            "upper": j["upper"],
            "max_velocity": j["max_velocity"],
            "max_effort": j["max_effort"],
        }

    def joint_limit_info(self, name):
        """Full joint information including type and links."""
        self._require_joint(name)
        return dict(self.joints[name])

    def report_joints(self):
        """Print a joint summary table (name, type, index, limits)."""
        print("%-28s %-10s %5s %8s %8s %8s %8s" % (
            "joint", "type", "idx", "lower", "upper", "vel_max", "eff_max"))
        for name in self.joint_names():
            j = self.joints[name]
            print("%-28s %-10s %5d %8.2f %8.2f %8.2f %8.1f" % (
                name, j["type_name"], j["index"],
                j["lower"], j["upper"], j["max_velocity"], j["max_effort"]))

    # ------------------------------------------------------------------ #
    # Mode handling
    # ------------------------------------------------------------------ #
    def set_control_mode(self, mode, joints=None):
        """Activate 'position' or 'velocity' control.

        Switching safely: every motor is first brought to a controlled stop
        (zero velocity command, holding force), then the new mode is armed.
        In position mode the position targets are re-initialised to the
        current joint positions so activation causes no jump.
        """
        if mode not in self._MODES:
            raise ValueError("mode must be 'position' or 'velocity', got %r" % mode)
        names = self._resolve_joints(joints, movable_only=True)
        # Controlled stop of everything first (prevents discontinuities).
        self.stop_all_motors(joints=names)
        self._mode = mode
        if mode == self.POSITION:
            for name in names:
                idx = self.joints[name]["index"]
                pos = p.getJointState(self.robot_id, idx, physicsClientId=self.client)[0]
                self._position_target[name] = pos
                # Push the hold command NOW: PyBullet would otherwise keep
                # applying the previous mode's last motor command.
                p.setJointMotorControl2(
                    self.robot_id, idx, p.POSITION_CONTROL,
                    targetPosition=pos, force=self._motor_force[name],
                    physicsClientId=self.client)
        else:
            for name in names:
                self._velocity_target[name] = 0.0
                self._velocity_command[name] = 0.0
                self._velocity_accel.setdefault(
                    name, 2.0 if ("finger" in name or "thumb" in name) else 1.0)
                # Zero-velocity command takes over from any latched position
                # command (safe: the joint holds still, no jump).
                p.setJointMotorControl2(
                    self.robot_id, self.joints[name]["index"], p.VELOCITY_CONTROL,
                    targetVelocity=0.0, force=self._motor_force[name],
                    physicsClientId=self.client)
        self._last_command_time = self.sim_time()
        return self._mode

    def get_control_mode(self):
        return self._mode

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def set_position_targets(
        self,
        targets,
        position_gain: float = 1.0,
        velocity_gain: float = 1.0,
        max_velocity: float = None,
        force: dict = None,
    ):
        """Command position targets for one or many joints.

        Args:
            targets: {joint_name: target_position_rad} or a single
                (joint_name, position) pair as ``{name: value}``.
            position_gain: servo strength for the position error (1.0 = crisp).
            velocity_gain: damping term of the internal servo.
            max_velocity: cap on joint speed while moving to the target
                (defaults to the URDF velocity limit).
            force: optional {joint_name: max_motor_torque} override.
        """
        self._require_mode(self.POSITION)
        targets = self._as_dict(targets)
        for name, target in targets.items():
            j = self.joints[name]
            clamped = self._clamp(target, j["lower"], j["upper"])
            if clamped != target:
                print("[SurenaVControl] %s: target %.3f clamped to limit %.3f"
                      % (name, target, clamped))
            vel_cap = max_velocity if max_velocity else j["max_velocity"]
            vel_cap = min(vel_cap, j["max_velocity"]) if max_velocity else j["max_velocity"]
            f = force.get(name, self._motor_force[name]) if force else self._motor_force[name]
            p.setJointMotorControl2(
                self.robot_id, j["index"], p.POSITION_CONTROL,
                targetPosition=clamped,
                force=f,
                positionGain=position_gain,
                velocityGain=velocity_gain,
                maxVelocity=vel_cap,
                physicsClientId=self.client)
            self._position_target[name] = clamped
        self._last_command_time = self.sim_time()

    def set_velocity_targets(
        self,
        targets,
        max_acceleration: float = None,
        force: dict = None,
    ):
        """Command velocity targets for one or many joints.

        The command is ACCELERATION-LIMITED: the actually applied velocity
        ramps toward the target at ``max_acceleration`` every simulation step
        (see ``step``), so a single call is enough and the ramp cannot be
        violated by late calls.

        Args:
            targets: {joint_name: target_velocity_rad_per_s}.
            max_acceleration: rad/s^2 ramp limit (defaults: 1.0 body /
                2.0 fingers -- conservative defaults, NOT verified actuator
                specs).
            force: optional {joint_name: max_motor_torque} override.
        """
        self._require_mode(self.VELOCITY)
        targets = self._as_dict(targets)
        for name, target in targets.items():
            j = self.joints[name]
            vel_cap = j["max_velocity"]
            clamped = self._clamp(target, -vel_cap, vel_cap)
            if clamped != target:
                print("[SurenaVControl] %s: velocity %.3f clamped to limit %.3f"
                      % (name, target, clamped))
            acc = max_acceleration if max_acceleration is not None else (
                2.0 if ("finger" in name or "thumb" in name) else 1.0)
            self._velocity_target[name] = clamped
            self._velocity_accel[name] = acc
            if force and name in force:
                self._motor_force[name] = float(force[name])
        self._last_command_time = self.sim_time()

    def move_to_positions(self, targets, duration: float = 2.0, **kwargs):
        """Blocking interpolated point-to-point move (cubic in time).

        Optional trajectory interpolation for position mode: the targets are
        approached along a cubic ease-in/ease-out profile over ``duration``
        seconds of simulated time.  If the profile would exceed the URDF
        velocity limit of any commanded joint, the duration is extended
        automatically (peak cubic velocity = 1.5 * distance / duration).
        """
        self._require_mode(self.POSITION)
        targets = self._as_dict(targets)
        start = {n: self.get_joint_positions([n])[n] for n in targets}
        # Respect velocity limits: extend the duration if needed.
        worst = 0.0
        for n, tgt in targets.items():
            dist = abs(tgt - start[n])
            if dist > 1e-9:
                worst = max(worst, 1.5 * dist / duration /
                            max(self.joints[n]["max_velocity"], 1e-6))
        if worst > 1.0:
            new_duration = duration * worst * 1.05
            print("[SurenaVControl] move duration extended %.2fs -> %.2fs to "
                  "respect URDF velocity limits" % (duration, new_duration))
            duration = new_duration
        steps = max(1, int(round(duration / self.time_step)))
        for s in range(1, steps + 1):
            a = self._cubic(s / steps)
            interp = {n: start[n] + a * (targets[n] - start[n]) for n in targets}
            self.set_position_targets(interp, **kwargs)
            self.step()

    # ------------------------------------------------------------------ #
    # Feedback
    # ------------------------------------------------------------------ #
    def get_joint_positions(self, names=None):
        """Joint positions (rad). Single name -> float; list/None -> dict."""
        out = {n: p.getJointState(self.robot_id, self.joints[n]["index"],
                                  physicsClientId=self.client)[0]
               for n in self._resolve_joints(names)}
        return out[names] if isinstance(names, str) else out

    def get_joint_velocities(self, names=None):
        """Joint velocities (rad/s). Single name -> float; list/None -> dict."""
        out = {n: p.getJointState(self.robot_id, self.joints[n]["index"],
                                  physicsClientId=self.client)[1]
               for n in self._resolve_joints(names)}
        return out[names] if isinstance(names, str) else out

    def get_joint_feedback(self, names=None):
        """Positions, velocities, reaction forces and applied motor torques.

        Reaction forces are the joint constraint forces reported by PyBullet
        (available for every joint); applied motor torque is the torque the
        motor last applied (available for actuated joints).
        Single name -> that joint's dict; list/None -> {name: dict}.
        """
        out = {}
        for n in self._resolve_joints(names):
            pos, vel, reaction, torque = p.getJointState(
                self.robot_id, self.joints[n]["index"], physicsClientId=self.client)[:4]
            out[n] = {
                "position": pos,
                "velocity": vel,
                "reaction_force": reaction,   # (Fx, Fy, Fz, Mx, My, Mz)
                "applied_torque": torque,
            }
        return out[names] if isinstance(names, str) else out

    # ------------------------------------------------------------------ #
    # Safety
    # ------------------------------------------------------------------ #
    def stop_all_motors(self, joints=None):
        """Controlled stop: ramp/hold to rest.

        In velocity mode the velocity targets are zeroed (the per-step ramp
        brings the joints to rest at the acceleration limit); in position
        mode the motors hold the last position target.
        """
        names = self._resolve_joints(joints, movable_only=True)
        for name in names:
            idx = self.joints[name]["index"]
            if self._mode == self.VELOCITY:
                self._velocity_target[name] = 0.0
            else:
                pos = p.getJointState(self.robot_id, idx, physicsClientId=self.client)[0]
                p.setJointMotorControl2(
                    self.robot_id, idx, p.POSITION_CONTROL,
                    targetPosition=pos, force=self._motor_force[name],
                    physicsClientId=self.client)
                self._position_target[name] = pos

    def emergency_stop(self):
        """Detach every motor immediately (free swing + gravity).

        Use only when a controlled stop is unsafe; the robot will collapse
        under gravity unless it is hanging or the base is fixed.
        """
        for name in self.movable_joint_names:
            p.setJointMotorControl2(
                self.robot_id, self.joints[name]["index"], p.VELOCITY_CONTROL,
                targetVelocity=0.0, force=0.0, physicsClientId=self.client)
        self._velocity_target = {n: 0.0 for n in self._velocity_target}
        self._velocity_command = {n: 0.0 for n in self._velocity_command}
        print("[SurenaVControl] EMERGENCY STOP: all motors detached.")

    def set_motor_force(self, force_map):
        """Override the motor force cap: {joint_name: N.m}."""
        for name, f in self._as_dict(force_map).items():
            self._motor_force[name] = float(f)

    def enable_watchdog(self, timeout: float = 0.5):
        """Fail-safe: if no command arrives for ``timeout`` seconds (sim time)
        while in velocity mode, ramp all commands to zero."""
        self._watchdog_timeout = timeout

    # ------------------------------------------------------------------ #
    # Simulation stepping
    # ------------------------------------------------------------------ #
    def sim_time(self):
        """Accumulated simulated time in seconds."""
        return self._sim_time

    def step(self, n: int = 1):
        """Advance the simulation ``n`` fixed steps.

        In velocity mode each step first ramps the applied command toward the
        stored target at the per-joint acceleration limit and pushes it to
        the motor, so acceleration limiting holds even for a single
        ``set_velocity_targets`` call followed by stepping.
        """
        for _ in range(n):
            self._sim_time += self.time_step
            self._apply_watchdog()
            if self._mode == self.VELOCITY:
                self._push_velocity_commands()
            p.stepSimulation(physicsClientId=self.client)

    def _push_velocity_commands(self):
        """Ramp applied velocity commands one step toward their targets."""
        dt = self.time_step
        for name, target in self._velocity_target.items():
            acc = self._velocity_accel.get(name, 1.0)
            prev = self._velocity_command.get(name, 0.0)
            ramped = prev + self._clamp(target - prev, -acc * dt, acc * dt)
            p.setJointMotorControl2(
                self.robot_id, self.joints[name]["index"], p.VELOCITY_CONTROL,
                targetVelocity=ramped, force=self._motor_force[name],
                physicsClientId=self.client)
            self._velocity_command[name] = ramped

    def run(self, seconds: float):
        """Step the simulation for ``seconds`` of simulated time."""
        self.step(int(round(seconds / self.time_step)))

    def close(self):
        if p.isConnected(self.client):
            p.disconnect(physicsClientId=self.client)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    @staticmethod
    def _find_package_root(urdf_path, package_root=None):
        """Locate the directory that CONTAINS the surenaV_description
        package (its meshes/ folder must hold the STL files).

        Search order:
          1. the explicit package_root argument,
          2. the default: parent-of-parent of this script (the catkin
             simulation/ directory in the original workspace),
          3. the URDF's own directory and up to four parents (covers layouts
             where surenaV_description sits next to or above the URDF).

        Raises FileNotFoundError with instructions if the meshes are missing.
        """
        mesh_probe = os.path.join(_MESH_PACKAGE, "meshes")

        candidates = []
        if package_root:
            candidates.append(os.path.abspath(package_root))
        candidates.append(os.path.abspath(_DEFAULT_PACKAGE_ROOT))
        d = os.path.dirname(os.path.abspath(urdf_path))
        for _ in range(5):
            candidates.append(d)
            d = os.path.dirname(d)

        for cand in candidates:
            if os.path.isdir(os.path.join(cand, mesh_probe)):
                return cand

        raise FileNotFoundError(
            "Surena V mesh files not found. The URDF references meshes as "
            "package://%s/meshes/*.STL, so the folder '%s' (containing "
            "'meshes/' with the 62 STL files) must exist in one of the "
            "searched locations:\n  %s\n"
            "Fix: run the build from the surenaV_core workspace (so the "
            "surenaV_description package sits next to surenaV_control), or "
            "pass SurenaVControl(..., package_root=r'<folder containing "
            "surenaV_description>')."
            % (_MESH_PACKAGE, _MESH_PACKAGE,
               "\n  ".join(candidates)))

    def _apply_watchdog(self):
        if (self._watchdog_timeout is None or self._mode != self.VELOCITY
                or self._last_command_time is None):
            return
        if self._sim_time - self._last_command_time > self._watchdog_timeout:
            stale = [n for n, v in self._velocity_command.items() if abs(v) > 1e-9]
            if stale:
                print("[SurenaVControl] watchdog: no command for %.2fs, "
                      "ramping %d joints to zero velocity" %
                      (self._watchdog_timeout, len(stale)))
                self.set_velocity_targets({n: 0.0 for n in stale})

    def _require_mode(self, mode):
        if self._mode is None:
            raise WrongControlModeError(
                "No control mode active; call set_control_mode('%s' or '%s') first."
                % (self.POSITION, self.VELOCITY))
        if self._mode != mode:
            raise WrongControlModeError(
                "Active mode is '%s' but the command is for '%s'. Switch modes "
                "with set_control_mode() first (this guard prevents two modes "
                "commanding the same joint)." % (self._mode, mode))

    def _require_joint(self, name):
        if name not in self.joints:
            raise UnknownJointError("Unknown joint %r; valid names: %s"
                                    % (name, ", ".join(self.joints)))

    def _resolve_joints(self, names, movable_only=False):
        if names is None:
            return list(self.movable_joint_names if movable_only
                        else self.joints.keys())
        if isinstance(names, str):
            names = [names]
        for n in names:
            self._require_joint(n)
        return list(names)

    @staticmethod
    def _as_dict(targets):
        if isinstance(targets, str):
            raise ValueError("targets must be {joint_name: value}; got a bare "
                             "string. Example: {'r_knee_joint': 0.5}")
        return dict(targets)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _cubic(s):
        """Cubic ease-in/ease-out interpolation factor, s in (0, 1]."""
        return s * s * (3.0 - 2.0 * s)
