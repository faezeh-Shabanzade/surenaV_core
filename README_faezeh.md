# surena_ws_updated — SURENA V with actuated hands in Gazebo (ROS 1 Noetic)

Complete catkin workspace based on the original `surena_ws`, extended with:

1. **One URDF for the whole body + 5-finger hands** (`surenaV_description/urdf/surenaV.urdf`,
   taken verbatim from the working `surenaV_hands_description` model — axes and frames
   untouched — with only the mesh paths repointed). The separate
   `surenaV_hands_description` package was merged into `surenaV_description`.
2. **Mimic-coupled distal finger joints** — the 5 distal joints per hand follow their
   driving joint 1:1, enforced in software by `gazebo_bridge` and `FingerControl`
   (the URDF itself carries no mimic tags, matching the working original).
3. **Finger actuation in Gazebo** with direct position controllers (no software PID —
   removed to eliminate finger trembling at rest). The 29 body joints keep their
   original effort+PID controllers, untouched.
4. **Individual finger control by joint name** via the `/set_joint_angle` rosservice.
5. **A standard set of timed finger scenarios** callable via `/finger_animation_srv`
   (and the single-shot `/finger_scenario_srv` for static poses).
6. **Arm scenarios with concurrent finger animations** via the optional
   `finger_scenario` field of `/move_hand_single_srv`.
7. **The left-hand mirroring fixes** (byebye, pointing, showHands, "a", wave direction,
   and the left-wrist Gazebo sign negation at channel 26), plus a numeric Jacobian /
   mirror audit that corrected the channel-28 (l_hand_pitch) Gazebo sign.
8. **Curve-drawing arm scenarios `circle` and `circle2`** — the palm (with the index
   finger extended) traces a circle in the air; both hands draw mirror-symmetric
   circles via `/move_hand_both_srv` (see section 3.5).
9. **Robust scenario input validation** — unknown scenario names or bad request
   fields return a clean service error instead of crashing the planner node.
10. **Inactive finger PID blocks** — commented effort+PID controller alternatives for
   all 22 finger joints, ready to uncomment (see section 4).
11. **Letter-drawing arm scenario `drawA`** — the pointing finger draws the letter
   "A" in a vertical plane in front of the body; single hand reads correctly from
   the front, both hands draw mirror-symmetric letters (see section 3.6).
12. **Gazebo reliability fixes** — `arm_home_service` no longer drifts the elbows
    forever in Gazebo; the left arm no longer jumps to a wrong pose when
    transitioning between scenarios in `/move_hand_both_srv`; the Gazebo
    `lower_thumb` joints flex the correct way on both hands (see sections 3.7
    and 4).

---

## 1. Build & run

```bash
cd ~/surena_ws_updated
catkin_make                 # or: catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# Terminal 1: Gazebo (spawns the robot; 29 body controllers with PID, 22 direct
#              position controllers for the fingers)
roslaunch surenaV_gazebo surenaV_gazebo.launch

# Terminal 2: robot manager (runs HandManager + GaitManager, exposes all services)
rosrun robot_manager robot_manager_node

# Terminal 3 (if not auto-started): the Gazebo bridge
# (fans /joint_angles_gazebo and /finger_angles_gazebo out to the per-joint controllers)
rosrun surenaV_gazebo gazebo_bridge
```

## 2. Finger joint model

Each hand has 11 revolute finger joints; 6 are actuated (one per hardware servo
channel), 5 follow their driving joint 1:1 (multiplier +1 on both hands — the
pristine SolidWorks frames already encode the left/right mirroring, so no axis
sign edits were needed):

| Joint (`<side>_` = `r_` or `l_`) | Type | Follows (mimic) | Multiplier |
|---|---|---|---|
| `middle_finger1..4_joint` | actuated (servo ch. 1–4) | — | — |
| `lower_thumb_joint` | actuated (servo ch. 5) | — | — |
| `middle_thumb_joint` | actuated (servo ch. 6) | — | — |
| `upper_finger1..4_joint` | software mimic | `middle_fingerN_joint` | +1 |
| `upper_thumb_joint` | software mimic | `middle_thumb_joint` | +1 |

Gazebo Classic does not enforce URDF `<mimic>` (and the URDF carries none), so the
coupling is applied in `gazebo_bridge` (defensively, on every command) and in
`FingerControl`. If a distal joint ever moves the wrong way on the robot, flip the
corresponding entry in `kMimicMultiplier`/`kMimicMultiplierL` (bridge and
`FingerControl.h`); if a whole channel flexes backward, flip its entry in
`kGazeboSignR`/`kGazeboSignL` (`FingerControl.h`, also used by `/set_joint_angle`).

**Servo ↔ angle mapping** (hardware convention, 6 channels per hand ordered
`[finger1, finger2, finger3, finger4, lower_thumb, middle_thumb]`):
`servo 0 = fully closed (flex = 1.5 rad)`, `servo 255 = fully open (0 rad)`:

```
angle_rad = (255 − servo) / 255 × 1.5
servo     = 255 − angle_rad / 1.5 × 255
```

Control bytes: `6` = right hand, `7` = left hand.

## 3. Services

### 3.1 Move one finger joint by name — `/set_joint_angle` (simulation)

```bash
# close the right index a bit (30 deg flex)
rosservice call /set_joint_angle "joint_name: 'r_middle_finger2_joint'" "angle_deg: 30.0"

# open the left thumb opposition channel
rosservice call /set_joint_angle "joint_name: 'l_lower_thumb_joint'" "angle_deg: 0.0"

# body joints work too
rosservice call /set_joint_angle "joint_name: 'r_elbow_pitch_joint'" "angle_deg: -45.0"
```

- Valid names: the 29 body joints + the 12 actuated finger joints.
- Mimic joints (`upper_*`) are rejected with a hint to command their driving joint.
- Finger angles are also mirrored into FingerControl's servo state, so any animation
  started afterwards begins from the new pose.
- Hardware mode: returns `success = false` (use `/finger_control_srv` for the servos).

### 3.2 Timed finger scenarios — `/finger_animation_srv`

```bash
# wave with the right hand fingers, 1 s per cycle, 4 cycles
rosservice call /finger_animation_srv "scenario_name: 'finger_wave'"
"hand_selection: 'right'" "duration: 1.0" "repetitions: 4"

# thumbs up with the left hand (moves in ~0.3 s, then holds)
rosservice call /finger_animation_srv "scenario_name: 'thumbs_up'"
"hand_selection: 'left'" "duration: 2.0" "repetitions: 1"
```

Standard animation scenarios (defined with keyframes in
`hand_planner/config/finger_scenarios.json`):

| Scenario | Motion |
|---|---|
| `finger_wave` | cyclic open ↔ half-closed (use `repetitions` > 1) |
| `point` | index extended, others curled |
| `power_grip` | all fingers curl into a fist |
| `pinch` | thumb + index tips meet |
| `thumbs_up` | fist with thumb up |
| `count_1` … `count_5` | progressively opening fingers |

Legacy static poses (`soft_grip`, `firm_grip`, `release`, `spherical`, `cylindrical`,
`writing`, `hook`, `pointing`, …) remain available through:

```bash
rosservice call /finger_scenario_srv "scenario_name: 'pinch'" "hand_selection: 'right'"
```

### 3.3 Direct servo control — `/finger_control_srv` (hardware + simulation)

```bash
# right hand: [finger1..4, lower_thumb, middle_thumb] servo values, control byte 6
rosservice call /finger_control_srv "target_positions: [250, 250, 80, 80, 60, 150]"
"control_data: 6" "hand_selection: 'right'"
```

### 3.4 Arm scenarios with concurrent finger animation

`/move_hand_single_srv` gained an optional `finger_scenario` field:

```bash
# right-arm 'byebye' while the right fingers wave
rosservice call /move_hand_single_srv "mode: 'righthand'" "ee_ini_pos: 'prev'"
"scenario: ['byebye']" "scen_count: 1" "t_total: 8" "finger_scenario: 'finger_wave'"

# left arm points while the left index points
rosservice call /move_hand_single_srv "mode: 'lefthand'" "ee_ini_pos: 'prev'"
"scenario: ['pointing']" "scen_count: 1" "t_total: 4" "finger_scenario: 'point'"
```

- `finger_scenario: ''` (default) keeps the previous behavior exactly.
- The finger animation runs on the hand being moved, at 200 Hz alongside the arm
  trajectory, and stops when the arm motion ends.
- All existing arm scenarios (`byebye`, `pointing`, `showHands`, `perfect`, `a`, `b`,
  …) are unchanged, including the left-hand mirroring fixes.

### 3.5 Curve-drawing arm scenarios — `circle` and `circle2`

New arm scenarios trace a circle in the air with the palm (index finger extended
if you pair them with `finger_scenario: 'point'`):

- `circle` — one full revolution on a vertical circle facing the robot
  (center 0.35 m forward, on the arm's own side at |y| = 0.16 m, z = -0.12 m,
  radius 7 cm). Duration ~28 s.
- `circle2` — two stacked loops (the second one raised by ~11 cm). Duration ~52 s.

The motion is built from the same proven min-jerk reach primitive used by every
other scenario, chained through 6 waypoints per revolution — numerically verified
in a Python port of the planner QP (waypoint error <= 0.2 mm per hand, exact
left/right mirror symmetry). The QP runs position-only during the trace: the
`pointing` palm orientation sits ~5 deg from an Euler-angle gimbal lock, and an
active orientation target there steals the whole velocity budget.

```bash
# both hands draw mirror-symmetric circles simultaneously (index fingers pointing)
rosservice call /move_hand_both_srv "scenR_count: 1" "scenL_count: 1"
"scenarioR: ['circle']" "scenarioL: ['circle']" "t_total: 60"

# single hand, with the index finger extended
rosservice call /move_hand_single_srv "mode: 'righthand'" "ee_ini_pos: 'prev'"
"scenario: ['circle']" "scen_count: 1" "t_total: 60" "finger_scenario: 'point'"

# two stacked loops, left hand
rosservice call /move_hand_single_srv "mode: 'lefthand'" "ee_ini_pos: 'prev'"
"scenario: ['circle2']" "scen_count: 1" "t_total: 60"
```

Notes:
- `circle`/`circle2` ignore `t_total` (their durations are fixed by the waypoint
  schedule); pass any positive value.
- Scenario names are **validated**: an unknown name (e.g. the typo `'poining'`)
  now returns a clean service error listing the valid names instead of crashing
  the planner node (`transport error`).
- Tuning constants (center, radius, waypoint count) sit at the top of
  `HandManager::reach_curve` in `hand_planner/src/HandManager.cpp`.

### 3.6 Letter-drawing arm scenario — `drawA`

`drawA` traces the letter **"A"** with the pointing finger in a vertical plane in
front of the body (x = 0.35 m). The letter is 14 cm tall x 10 cm wide; the right
hand draws at y = -0.10 m so it reads correctly for an observer facing the robot
(bottom-left -> apex -> bottom-right, then the crossbar left -> right). The left
hand traces the exact y-mirror, so `/move_hand_both_srv` draws two
mirror-symmetric "A"s simultaneously.

The trace reuses the verified waypoint-chained min-jerk tracer of `circle`
(position-only QP — see section 3.5 for why). Numerically verified in the Python
port: waypoint arrival error <= 0.5 mm per hand, exact left/right mirror
symmetry, and the whole letter stays within 0.3 mm of the drawing plane.

Duration is fixed at ~24 s (2 s joint prep + 2 s hold, then 5 chained 4-s
segments: the approach plus the 4 strokes); `t_total` is ignored — pass any
positive value.

The pointing finger pose is applied automatically:

- `/move_hand_single_srv`: if you leave `finger_scenario` empty, the `point`
  finger animation runs for the trace duration; an explicit `finger_scenario`
  always wins.
- `/move_hand_both_srv` has no `finger_scenario` field; the static `point` pose
  is applied to each drawing hand before the trace starts.

```bash
# both hands draw mirror-symmetric "A"s
rosservice call /move_hand_both_srv "scenR_count: 1" "scenL_count: 1"
"scenarioR: ['drawA']" "scenarioL: ['drawA']" "t_total: 30"

# right hand only, letter readable from the front
rosservice call /move_hand_single_srv "mode: 'righthand'" "ee_ini_pos: 'prev'"
"scenario: ['drawA']" "scen_count: 1" "t_total: 30" "finger_scenario: ''"

# left hand (mirrored variant)
rosservice call /move_hand_single_srv "mode: 'lefthand'" "ee_ini_pos: 'prev'"
"scenario: ['drawA']" "scen_count: 1" "t_total: 30"
```

Tuning constants (plane, letter size, crossbar height) sit at the top of
`HandManager::reach_drawA` in `hand_planner/src/HandManager.cpp`.

### 3.7 Other services

`/move_hand_both_srv`, `/execute_scenario_srv` (YAML scenario player), `/arm_home_service`,
`/hjoints_sens_read_srv`, `/joints_sens_read_srv`, `/gjoints_sens_read_srv`,
`/head_track_srv`, `/teleoperation_srv`, etc.

**`/arm_home_service` fix (Gazebo):** the hall-sensor homing state machine waits
for bits of `/surena/hall_state`, which nothing publishes in Gazebo — the elbows
used to ramp backward forever. If no hall-sensor message has ever arrived, the
service now blends the arm commands smoothly (3 s, smoothstep) from their
current value down to zero, which puts both arms at the initial (spawn) pose in
a **single call**, and then resets the tracked state/baselines to the
node-start values so subsequent scenarios behave exactly like after launching
Gazebo. (Earlier revisions blended toward a non-zero offset, which read as
"arms to the back" and needed a second call.) On the real robot (hall sensors
alive) the original homing sequence runs unchanged.

**Finger gestures now hold until the arm scenario ends** (single-hand
`finger_scenario`, and the auto-pointing of `drawA`): the finger animation runs
for the exact arm trajectory duration — `byebye` adds a 4 s wave, `shakeHands`
5 s, scenario lists run back-to-back, and `circle`/`circle2`/`drawA` ignore
`t_total`, so the old `t_total`-based timing released the gesture early.
Hold-type gestures (e.g. `point`) play once and are then held; periodic ones
(e.g. `finger_wave`) repeat until the end. When the animation expires, the
final gesture pose is applied once and kept by the position controllers.

`/move_hand_both_srv`, `/execute_scenario_srv` (YAML scenario player), `/arm_home_service`,
`/hjoints_sens_read_srv`, `/joints_sens_read_srv`, `/gjoints_sens_read_srv`,
`/head_track_srv`, `/teleoperation_srv`, etc.

## 4. Gazebo command path

```
HandManager (200 Hz control loop)
  ├─ /joint_angles_gazebo  (29 body values)  ──► RobotManager (publish trigger)
  └─ /finger_angles_gazebo (22 finger values: right 0–10, left 11–21,
        [middle_finger1..4, lower_thumb, middle_thumb, upper_finger1..4, upper_thumb])
        ──► gazebo_bridge ──► /surenaV/<joint>_position_controller/command
```

The per-channel sign table `kGazeboSignR`/`kGazeboSignL`
(`hand_planner/include/FingerControl.h`) is applied to the Gazebo finger commands
only (hardware servo bytes are untouched). The URDF `lower_thumb` axes are
handed (right axis z < 0, left axis z > 0), so with the shared flex-angle
convention the Gazebo `lower_thumb` joints rotated the opposite way to the
hardware servos on **both** hands; the `lower_thumb` entry (index 4) is now
`-1` in both tables. `/set_joint_angle` reads the same tables, so both command
paths are corrected by that single change.

- The **29 body joints** use `effort_controllers/JointPositionController` with the
  original PID gains in `surenaV_gazebo/config/surenaV_control.yaml` (untouched).

- The **22 finger joints** currently use direct
  `position_controllers/JointPositionController` entries (no PID — this removed the
  finger trembling at rest). **Inactive effort+PID alternatives are pre-written and
  commented out** directly beneath each finger controller in
  `surenaV_gazebo/config/surenaV_control.yaml` and
  `surenaV_control/config/position_controllers.yaml`, so you can activate them later:
  1. Comment out the finger's position block; uncomment the `effort_controllers/`
     block + `pid:` line beneath it.
  2. In the URDF, change that joint's `<transmission>` `<hardwareInterface>` from
     `PositionJointInterface` back to `EffortJointInterface` (Gazebo's hardware sim
     reads the first interface tag).
  3. Rebuild + relaunch. Suggested starting gains: `p: 100.0, i: 0.1, d: 5.0`.
- The **22 finger joints** use `position_controllers/JointPositionController` — a direct
  position interface with **no software PID**. The previous finger PID (p=100, i=0.1,
  d=5.0) caused trembling at rest; the corresponding URDF transmissions declare
  `hardware_interface/PositionJointInterface` so Gazebo's `DefaultRobotHWSim` serves
  them with its internal stiff servo. Body transmissions remain
  `EffortJointInterface`.

### Left/right mirror audit (numeric)

A numeric audit of the planner Jacobian and the Gazebo command path verified:

- The analytic `J_palm` in `S5_hand_jacobian.cpp` matches numeric differentiation of the
  FK to ~1e-11 for **both** hands (shoulder-fix +10° right / −10° left) — the Jacobian
  itself is correct.
- The left FK is the exact y-mirror of the right at the per-joint sign relation
  D = (+1, −1, −1, +1, −1, −1, +1) (pitch same; roll, elbow, forearm_link, hand_roll
  opposite; hand_pitch same).
- Combined with the URDF axis relations, the required left-channel Gazebo signs are
  (+1, +1, +1, +1, −1, +1, **+1**). The old channel-28 (`l_hand_pitch`) negation was
  wrong and broke the mirror as soon as the wrist pitched (error up to ~1.2 in
  position+orientation); it now passes through unchanged (residual mirror error
  ~2 mm, from the real geometry not being a perfect mathematical mirror).
  Channel 26 (`l_forearm_link`) keeps its negation.

- **Scenario-transition fix (left arm jumping):** `both_hands` re-based the left
  arm's command offset on the previous scenario's end pose on every call, while
  the right arm kept one constant baseline. Since arm channels are sent as
  `(absolute - baseline)`, the second `move_hand_both` call commanded the left
  arm to near-zero joint values (a visible collapse/jump, e.g. pointing ->
  byebye). Both arms now share the same set-once baseline rule, and the port
  simulation shows the left channels are an exact D-mirror of the right ones
  during the whole transition (max deviation 2.6 deg, the QP's normal hand
  asymmetry).

## 5. Workspace layout (relevant parts)

```
src/surenaV_core/
├── hand_planner/
│   ├── include/FingerControl.h        # servo/angle/mimic math + timed animations
│   ├── include/HandManager.h
│   ├── src/FingerControl.cpp          # applyServoPositions, startAnimation/tickAnimation
│   ├── src/HandManager.cpp            # services, arm scenarios, mirroring fixes
│   ├── config/finger_scenarios.json   # static poses + keyframe animations
│   └── srv/FingerAnimation.srv
├── robot_manager/
│   ├── src/RobotManager.cpp           # /set_joint_angle, finger_gazebo_pub_
│   └── include/RobotManager.h
├── simulation/
│   ├── surenaV_description/
│   │   ├── urdf/surenaV.urdf          # full body + hands (pristine SolidWorks model)
│   │   └── meshes/                    # body + hand STLs (62 files)
│   ├── surenaV_gazebo/
│   │   ├── src/gazebo_bridge.cpp      # body + finger fan-out, mimic re-enforcement
│   │   ├── config/surenaV_control.yaml
│   │   └── launch/
│   └── surenaV_control/
└── surena_msgs/srv/SetJointAngle.srv
```

## 6. Known limitations

- The workspace is **compile-ready but was not run in a live Gazebo** (no ROS in the
  build environment used to produce it). First-run tuning items:
  - Finger **channel direction in Gazebo**: if a channel flexes the wrong way, flip its
    entry in `kGazeboSignR`/`kGazeboSignL` (`hand_planner/include/FingerControl.h`);
    if only a distal joint is wrong, flip its `kMimicMultiplier` entry (also in
    `gazebo_bridge.cpp`). All entries default to +1 except `lower_thumb`
    (index 4, set to -1 on both hands — see section 4).
  - Finger joints use direct position controllers (no PID). If a finger sags under
    load, that is the trade-off of removing the software PID; body PID is unaffected.
  - `kMaxFlexRad = 1.5` rad is the assumed fully-closed flex angle (URDF limits are
    ±1.57 rad).
- Gazebo Classic does not honor URDF `<mimic>` (the URDF carries none); coupling is
  enforced in software (bridge + FingerControl). If a command bypasses both, distal
  joints will not follow.
- `/set_joint_angle` is simulation-only; hardware fingers go through the servo protocol
  (`/finger_control_srv`).
- `perfect` and `b` left-hand scenarios intentionally keep their original (non-mirrored)
  targets: their mirrored pose needs the left elbow at +90°, outside the joint limits
  ([-90°, -5°] on both arms).
