#!/usr/bin/env python3
"""Comprehensive arm-movement demo for the Surena V in PyBullet.

Moves the 7-joint right/left/both arms through a full position-control and
velocity-control program, on top of the SurenaVControl manager:

  POSITION phase   named poses (home, reach forward, reach side, wave)
                   executed with cubic interpolation that respects the URDF
                   velocity limits, with per-pose tracking-error readout.
  VELOCITY phase   ramped velocity commands: a sinusoidal shoulder-pitch
                   sweep and a constant-velocity elbow sweep, then a
                   controlled ramp to zero.
  MIRROR phase     (only with --arm both) the same program mirrored onto the
                   other arm: pitch angles are kept, roll angles negated.

All angles are demo choices inside the URDF's +/-1.57 rad joint limits; the
manager additionally clamps every command to the limits.

Usage:
    python3 demo_arm_movement.py                       # headless, full program
    python3 demo_arm_movement.py --gui                 # watch it move
    python3 demo_arm_movement.py --arm both --gui
    python3 demo_arm_movement.py --mode position       # position phase only
    python3 demo_arm_movement.py --mode velocity
    python3 demo_arm_movement.py --speed 0.5           # slower, gentler

Exit code 0 iff every position pose was reached within tolerance and no
command exceeded a joint limit.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pybullet_control_manager import SurenaVControl  # noqa: E402

# The 7 arm joints per side (verified against the URDF and the controller
# configs; the same list the r_arm/l_arm trajectory controllers use).
ARM_JOINTS = {
    "r": ["r_arm_pitch_joint", "r_arm_roll_joint", "r_elbow_pitch_joint",
          "r_forearm_roll_joint", "r_forearm_link_joint",
          "r_hand_pitch_joint", "r_hand_roll_joint"],
    "l": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_elbow_pitch_joint",
          "l_forearm_roll_joint", "l_forearm_link_joint",
          "l_hand_pitch_joint", "l_hand_roll_joint"],
}

# Joint-name fragments that change sign when a pose is mirrored onto the
# other arm (standard humanoid convention; pitch values are kept as-is).
_MIRROR_NEGATE = ("roll",)

# Named poses, keyed by joint-name fragment. Values are radians.
# "home" is explicit (all arm joints to zero) so it produces a real
# tracking-error readout instead of an empty command set.
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

# Position-phase program: (pose, hold seconds)
POSITION_PROGRAM = [
    ("home", 0.5), ("reach_forward", 1.0), ("reach_side", 1.0),
    ("home", 0.5), ("wave_up", 1.0), ("wave_out", 0.8), ("wave_up", 0.8),
    ("wave_out", 0.8), ("home", 1.0),
]

# Velocity-phase parameters (rad/s and Hz) -- well inside the 0.5 rad/s
# URDF body limit; --speed scales them.
VEL_SWEEPS = [
    # (label, joint fragment, amplitude rad/s, frequency Hz, duration s)
    ("shoulder-pitch sine sweep", "arm_pitch", 0.20, 0.25, 8.0),
    ("elbow constant sweep", "elbow_pitch", 0.15, None, 6.0),
    ("forearm-roll sine sweep", "forearm_roll", 0.15, 0.3, 6.0),
]

TOLERANCE = 0.05  # rad; pose reached if every commanded joint is within this


def build_pose(side, fragments):
    """Expand {fragment: angle} into {full joint name: angle} for one arm."""
    out = {}
    for frag, angle in fragments.items():
        # fragments are stored without the side prefix, e.g. "arm_pitch"
        # matches "r_arm_pitch_joint"; rebuild the full name.
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
    out = {}
    for joint, angle in pose.items():
        if any(frag in joint for frag in _MIRROR_NEGATE):
            out[joint] = -angle
        else:
            out[joint] = angle
    return out


def clamp_to_limits(bot, targets):
    """Clamp a {joint: angle} dict into each joint's URDF limits."""
    clamped = {}
    for joint, angle in targets.items():
        lim = bot.joint_limits(joint)
        clamped[joint] = max(lim["lower"], min(lim["upper"], angle))
        if clamped[joint] != angle:
            print("   ! %s clamped %.2f -> %.2f (URDF limit)"
                  % (joint, angle, clamped[joint]))
    return clamped


def run_position_phase(bot, sides, speed):
    """Named-pose program with cubic interpolation and error readout."""
    print("\n" + "=" * 66)
    print("PHASE 1: POSITION CONTROL (cubic interpolation, URDF limits)")
    print("=" * 66)
    bot.set_control_mode("position")
    worst = 0.0
    for pose_name, hold in POSITION_PROGRAM:
        targets = {}
        for side in sides:
            pose = build_pose(side, POSES[pose_name])
            targets.update(pose if side == "r" else mirror(side, pose))
        targets = clamp_to_limits(bot, targets)
        # Duration scales with the largest travel; move_to_positions
        # auto-extends it if the cubic profile would exceed the URDF
        # velocity limit.
        max_dist = max((abs(targets[n] - bot.get_joint_positions(n))
                        for n in targets), default=0.0)
        duration = max(1.0, max_dist / (0.25 * speed))
        bot.move_to_positions(targets, duration=duration)
        bot.run(hold / speed)
        errors = {n: abs(bot.get_joint_positions(n) - targets[n])
                  for n in targets}
        worst = max(worst, max(errors.values()))
        print("  %-14s max |error| = %.4f rad%s"
              % (pose_name, max(errors.values()),
                 "  (OK)" if max(errors.values()) < TOLERANCE else "  (DRIFT)"))
    return worst


def run_velocity_phase(bot, sides, speed):
    """Ramped velocity sweeps, then a controlled stop."""
    print("\n" + "=" * 66)
    print("PHASE 2: VELOCITY CONTROL (ramped commands, URDF limits)")
    print("=" * 66)
    bot.set_control_mode("velocity")
    dt = bot.time_step
    for label, frag, amplitude, freq, duration in VEL_SWEEPS:
        joints = {side: build_pose(side, {frag: 0.0}) for side in sides}
        # start every sweep from zero velocity
        bot.set_velocity_targets({j: 0.0 for d in joints.values() for j in d})
        print("\n  %s  (%.2f rad/s%s, %.1f s)"
              % (label, amplitude * speed,
                 ", %.2f Hz" % freq if freq else ", constant", duration))
        steps = int(duration * speed / dt)
        update_every = max(1, int(0.05 / dt))  # refresh targets at 20 Hz
        peak = 0.0
        for i in range(steps):
            t = i * dt
            for side, template in joints.items():
                if freq is None:
                    # constant velocity, reversing every 2 s
                    v = amplitude * speed if (t // 2.0) % 2 == 0 else -amplitude * speed
                else:
                    v = amplitude * speed * math.sin(2.0 * math.pi * freq * t)
                if side == "l" and "roll" in frag:
                    v = -v  # mirrored roll convention
                if i % update_every == 0:
                    bot.set_velocity_targets({j: v for j in template})
            bot.step()
            peak = max(peak, max(abs(v) for v in
                                 bot.get_joint_velocities(
                                     list(joints["r"])).values()))
        # decelerate to zero before the next sweep
        bot.set_velocity_targets({j: 0.0 for d in joints.values() for j in d})
        bot.run(1.0 / speed)
        fb = bot.get_joint_velocities(list(joints["r"]))
        print("     peak |velocity| %.3f rad/s (command amplitude %.2f); "
              "after stop: %s"
              % (peak, amplitude * speed,
                 ", ".join("%.3f" % abs(v) for v in fb.values())))
    # leave the arm held at the current pose
    bot.set_control_mode("position")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gui", action="store_true",
                        help="open the PyBullet GUI")
    parser.add_argument("--free-base", action="store_true",
                        help="leave the base free-floating. Default is a "
                             "pinned base: sweeping the shoulder of a "
                             "free-standing robot rocks the whole body and "
                             "the measured joint velocity spikes far beyond "
                             "the command (measured 4 rad/s vs 0.2 rad/s).")
    parser.add_argument("--arm", choices=["r", "l", "both"], default="r",
                        help="which arm(s) to move (default: r)")
    parser.add_argument("--mode", choices=["position", "velocity", "both"],
                        default="both", help="which phases to run")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="scale factor: <1 slower/gentler, >1 faster")
    args = parser.parse_args()

    sides = ["r", "l"] if args.arm == "both" else [args.arm]
    bot = SurenaVControl(gui=args.gui, fixed_base=not args.free_base)
    print("Surena V arm-movement demo")
    print("  arm(s): %s   mode: %s   speed: %.2f" %
          (args.arm, args.mode, args.speed))
    print("  arm joints: %s" % ", ".join(ARM_JOINTS["r"]))

    worst = 0.0
    try:
        if args.mode in ("position", "both"):
            worst = run_position_phase(bot, sides, args.speed)
        if args.mode in ("velocity", "both"):
            run_velocity_phase(bot, sides, args.speed)
            # return to home so the demo ends in a known pose
            home = {}
            for side in sides:
                home.update(build_pose(side, POSES["home"]))
            bot.move_to_positions(home, duration=2.0 / args.speed)
    except KeyboardInterrupt:
        print("\ninterrupted -- emergency stop")
        bot.emergency_stop()
        return 1
    finally:
        bot.close()

    print("\n" + "=" * 66)
    print("RESULT: %s (worst pose error %.4f rad, tolerance %.2f)"
          % ("PASS" if worst < TOLERANCE else "FAIL", worst, TOLERANCE))
    return 0 if worst < TOLERANCE else 1


if __name__ == "__main__":
    sys.exit(main())
