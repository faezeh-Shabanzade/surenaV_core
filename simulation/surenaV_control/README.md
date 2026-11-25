# surenaV_control

Position and velocity control for the **Surena V** humanoid (with five-finger
hands), built on the merged `surenaV_description` model and the
`surenaV_gazebo` Gazebo Classic integration.

One consistent control architecture, three backends:

| Backend | Position control | Velocity control | Mode switching |
|---|---|---|---|
| **Gazebo Classic** (ROS 1 Noetic) | `position_controllers/JointPositionController` over `hardware_interface/PositionJointInterface` | `velocity_controllers/JointVelocityController` over `hardware_interface/VelocityJointInterface` | relaunch with the other URDF variant (`launch/switch_control_mode.sh`); controller-to-controller switching via `controller_manager/switch_controller` |
| **PyBullet** (standalone Python) | `p.POSITION_CONTROL` via `set_position_targets()` | `p.VELOCITY_CONTROL` via `set_velocity_targets()` | `set_control_mode("position"/"velocity")` at runtime |
| **Physical robot** (template) | `hardware_interface::RobotHW` forwarding to the EPOS4 internal position loop (CSP) | same, velocity loop (PVM) | `switchControlMode()` in the RobotHW template |

## Design decisions

**No PID loops in ROS.** The physical actuators (Maxon EPOS4) run internal
position/velocity loops. The ROS controllers used here are *direct* command
interfaces: `position_controllers/JointPositionController` writes the position
command straight through to the actuator (in Gazebo Classic:
`DefaultRobotHWSim`'s ideal `SetAngle` servo, torque-capped by the URDF effort
limit). This replaces the previous `effort_controllers/JointPositionController`
+ PID-gains setup, which put a software PID on top of the effort interface.

**Two generated URDF variants, one mechanical source.** Gazebo Classic's
`DefaultRobotHWSim` reads only the *first* `<hardwareInterface>` tag of each
transmission, so one URDF cannot expose both command interfaces. The
mechanical model stays single-source
(`surenaV_description/urdf/surenaV.urdf`, never edited);
`scripts/generate_urdf_variants.py` (run automatically by `catkin_make`)
rewrites only the interface tags:

- `urdf/surenaV_position.urdf` — all 51 transmissions → `PositionJointInterface`
- `urdf/surenaV_velocity.urdf` — all 51 transmissions → `VelocityJointInterface`

Everything else (links, joints, limits, inertials, meshes, Gazebo tags) is
byte-identical to the source; `scripts/validate_urdf_and_configs.py` proves it.

**Why not a combined-interface plugin?** A custom `RobotHWSim` could register
both interfaces and switch at runtime, but it adds a C++ plugin to build and
maintain for one convenience. The two-variant approach uses only stock
packages. The trade-off is documented in the final report; the RobotHW
template for the real robot *does* register both interfaces (there the driver
is custom anyway).

## Package layout

```
surenaV_control/
├── config/
│   ├── position_controllers.yaml     # joint_state_controller + 51 position controllers
│   ├── velocity_controllers.yaml     # joint_state_controller + 51 velocity controllers
│   ├── trajectory_controllers.yaml   # 7 group-wise JointTrajectoryControllers (position session)
│   └── joint_limits.yaml             # per-joint limits; placeholders flagged
├── launch/
│   ├── gazebo_position.launch        # Gazebo Classic + position session
│   ├── gazebo_velocity.launch        # Gazebo Classic + velocity session
│   ├── pybullet_position.launch      # PyBullet + ROS topics, position
│   ├── pybullet_velocity.launch      # PyBullet + ROS topics, velocity
│   └── switch_control_mode.sh        # position <-> velocity relaunch helper
├── scripts/
│   ├── pybullet_control_manager.py   # standalone PyBullet control library (no ROS)
│   ├── pybullet_position_control.py  # demo: single/multi/sequence position moves
│   ├── pybullet_velocity_control.py  # demo: velocity moves, ramping, watchdog
│   ├── switch_control_mode.py        # demo: safe mode switching + guards
│   ├── pybullet_ros_node.py          # ROS wrapper mirroring the Gazebo topics
│   ├── generate_urdf_variants.py     # builds the two URDF variants
│   └── validate_urdf_and_configs.py  # static validation (no ROS needed)
├── src/hardware_interface/           # RobotHW TEMPLATE for the real robot
│   ├── surenaV_hw_interface.{h,cpp}
│   ├── epos_driver_stub.h            # actuator seam (NOT implemented)
│   └── HARDWARE_INFO_NEEDED.md       # exact facts required to finish it
├── urdf/                             # generated variants (do not edit)
├── test/
│   ├── test_joint_mapping.py
│   ├── test_position_control.py
│   └── test_velocity_control.py
├── CMakeLists.txt
└── package.xml
```

## Install and build

On the ROS Noetic machine (Gazebo Classic 11), inside `surena_ws`:

```bash
cd ~/surena_ws/src/surenaV_core/simulation
# place this package here (it ships inside surenaV_core/simulation/)
cd ~/surena_ws
catkin_make            # regenerates urdf/surenaV_{position,velocity}.urdf
source devel/setup.bash
```

PyBullet (simulation only, no ROS needed):

```bash
pip install pybullet
```

## Gazebo Classic usage

```bash
# POSITION session (robot spawns paused; press Play in the GUI)
roslaunch surenaV_control gazebo_position.launch

# send a position command to one joint (rad)
rostopic pub -1 /surenaV/r_elbow_pitch_joint_position_controller/command \
    std_msgs/Float64 "data: 0.5"

# read feedback (all 51 joints, 100 Hz)
rostopic echo /surenaV/joint_states

# coordinated multi-joint motion: activate a group trajectory controller
rosservice call /surenaV/controller_manager/switch_controller \
    "{start_controllers: [r_arm_trajectory_controller], \
      stop_controllers: [r_arm_pitch_joint_position_controller, \
                         r_arm_roll_joint_position_controller, \
                         r_elbow_pitch_joint_position_controller, \
                         r_forearm_roll_joint_position_controller, \
                         r_forearm_link_joint_position_controller, \
                         r_hand_pitch_joint_position_controller, \
                         r_hand_roll_joint_position_controller], \
      strictness: 1}"
rosaction call /surenaV/r_arm_trajectory_controller/follow_joint_trajectory \
    "trajectory: {joint_names: [r_arm_pitch_joint, r_arm_roll_joint, \
        r_elbow_pitch_joint, r_forearm_roll_joint, r_forearm_link_joint, \
        r_hand_pitch_joint, r_hand_roll_joint], \
      points: [{positions: [-0.3, 0.0, -0.9, 0.4, 0.0, 0.0, 0.0], \
                time_from_start: 3.0}]}"

# VELOCITY session (separate launch; commands are rad/s)
roslaunch surenaV_control gazebo_velocity.launch
rostopic pub -1 /surenaV/r_elbow_pitch_joint_velocity_controller/command \
    std_msgs/Float64 "data: 0.2"

# switch the whole session position <-> velocity
rosrun surenaV_control switch_control_mode.sh velocity
```

Rules the switch sequence enforces:

- Never run two controllers that command the same joint (stop the previous
  one in the same `switch_controller` call; `strictness: 1` = best effort,
  use `2` to abort if a stop fails).
- Commands start at the safe state: position controllers latch 0.0 (the
  robot's home pose), velocity controllers command 0 rad/s.
- Switching the *session* mode (position ↔ velocity) requires the relaunch
  because the hardware sim registers one interface per joint.

## PyBullet usage (standalone)

```python
from surenaV_control.scripts import pybullet_control_manager  # or import by path
from pybullet_control_manager import SurenaVControl

bot = SurenaVControl(gui=True)                 # loads the Surena V URDF
bot.report_joints()                            # names, types, indices, limits

bot.set_control_mode("position")
bot.move_to_positions({"r_elbow_pitch_joint": 0.5}, duration=2.0)
bot.set_position_targets({"r_knee_joint": 0.3, "l_knee_joint": 0.3})

bot.set_control_mode("velocity")               # safe stop + re-arm inside
bot.set_velocity_targets({"r_elbow_pitch_joint": 0.2})
bot.run(1.0)

bot.stop_all_motors()                          # controlled stop
bot.emergency_stop()                           # detach all motors
bot.close()
```

Demos (headless by default, `--gui` to watch):

```bash
python3 scripts/pybullet_position_control.py
python3 scripts/pybullet_velocity_control.py
python3 scripts/switch_control_mode.py
```

PyBullet does **not** read URDF transmission tags; the control mode is set
explicitly in Python (`p.POSITION_CONTROL` / `p.VELOCITY_CONTROL`), and the
mode guard makes it impossible for both modes to command a joint at once.

## PyBullet + ROS

```bash
roslaunch surenaV_control pybullet_position.launch   # or pybullet_velocity.launch
rostopic pub -1 /surenaV/r_elbow_pitch_joint_position_controller/command \
    std_msgs/Float64 "data: 0.5"
rostopic echo /surenaV/joint_states
```

The node publishes/subscribes the same topic names as the Gazebo session, so
high-level nodes (e.g. `robot_manager` + `gazebo_bridge`) run against either
simulator unchanged.

## Arm-movement demos

Complete arm programs (named poses, sweeps, mirrored two-arm sequences) for
the 7-joint arms, in both control modes and both simulators.

PyBullet, direct API (no ROS needed):

```bash
python3 scripts/demo_arm_movement.py                    # headless, full program
python3 scripts/demo_arm_movement.py --gui              # watch it move
python3 scripts/demo_arm_movement.py --arm both --gui   # mirrored two arms
python3 scripts/demo_arm_movement.py --mode velocity    # sweeps only
python3 scripts/demo_arm_movement.py --speed 0.5        # slower, gentler
```

Phases: **position** — home / reach-forward / reach-side / wave poses with
cubic interpolation and per-pose tracking-error readout; **velocity** —
sinusoidal shoulder sweep, constant-velocity elbow sweep, forearm-roll sweep,
each with soft start/stop. The base is pinned by default (`--free-base` to
release it): sweeping the shoulder of a free-standing robot rocks the whole
body and the measured joint velocity spikes far beyond the command.

ROS 1 (works against Gazebo Classic *or* the PyBullet bridge — same topics):

```bash
# one command each: starts the simulator session + the demo node
roslaunch surenaV_control arm_demo_position.launch                # Gazebo, position
roslaunch surenaV_control arm_demo_velocity.launch                # Gazebo, velocity
roslaunch surenaV_control arm_demo_pybullet.launch                # PyBullet, position
roslaunch surenaV_control arm_demo_pybullet.launch mode:=velocity
# options: _arm:=r|l|both  _speed:=0.5

# or run the node against an already-running session:
rosrun surenaV_control arm_movement_demo.py _mode:=position _arm:=both
```

Smooth multi-waypoint trajectories through the group trajectory controller
(position session only — ROS 1 `joint_trajectory_controller` supports the
PositionJointInterface only). The node activates the stopped trajectory
controller itself via `controller_manager/switch_controller`:

```bash
roslaunch surenaV_control gazebo_position.launch
rosrun surenaV_control arm_trajectory_demo.py --arm r
```

## Tests

```bash
cd test
python3 test_joint_mapping.py       # URDF <-> PyBullet consistency (10 tests)
python3 test_position_control.py    # tracking, clamping, switching (8 tests)
python3 test_velocity_control.py    # ramping, watchdog, e-stop (9 tests)
python3 ../scripts/validate_urdf_and_configs.py   # static checks, no ROS needed
```

All 27 tests pass headless in this repository (PyBullet 3.2.7, Python 3.11).
The Gazebo/ROS launch files are validated statically only — no ROS
installation was available where this package was developed; run the launch
commands above on the Noetic machine.

## Real-robot roadmap

1. Fill in `src/hardware_interface/epos_driver_stub.h` using the existing
   `surena_eth` EPOS4/CAN-UDP code; the required facts are listed in
   `src/hardware_interface/HARDWARE_INFO_NEEDED.md`.
2. Uncomment the hardware-interface block in `CMakeLists.txt`.
3. Write a `surenaV_hardware` node that loads `SurenaVHardwareInterface`
   into a `controller_manager`, reusing the same controller YAMLs.
4. Keep using `controller_manager/switch_controller` for controller changes
   and `switchControlMode()` (stop controllers first) for actuator mode
   changes. Switching the ROS controller does **not** by itself change the
   EPOS4 operating mode — the hardware interface does that explicitly.

## Known limitations / unverified values

- `effort=1000` (body) and `velocity=0.5 rad/s` in the URDF look like export
  placeholders; they are used as-is and flagged in `config/joint_limits.yaml`.
  Real actuator limits are unknown (see HARDWARE_INFO_NEEDED.md).
- Acceleration limits are not in the URDF; the PyBullet ramping defaults
  (1.0 body / 2.0 fingers rad/s²) are conservative defaults, not specs.
- `joint_trajectory_controller` in ROS 1 is compiled for the position
  interface only, so trajectory controllers exist in the position session.
- The finger joints are individually actuated in the model; the physical hand
  couples 22 joints to 6 actuators per hand (mapping documented in
  `gazebo_bridge.cpp`).
