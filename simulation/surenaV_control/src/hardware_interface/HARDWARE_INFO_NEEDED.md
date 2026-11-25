# Hardware information needed to finish the driver

The `hardware_interface::RobotHW` template (`surenaV_hw_interface.{h,cpp}`) is
complete except for the actuator communication layer (`epos_driver_stub.h`).
Per the project rules the protocol is **not** invented; the existing
`surena_eth` package already speaks it for the Qt stack and is the reference
implementation. To finish `EposDriverStub`, provide/confirm the following.

## 1. Transport

| Fact | Needed for | Where it likely lives |
|---|---|---|
| UDP packet layout (header, node addressing, checksum) | framing read/write | `surena_eth/src/ethernetpackets.h`, `udphandler.cpp` |
| Bridge IP address / port(s), one socket or per-node? | connection setup | `surena_eth/src/can.cpp` |
| Poll rate of the feedback broadcast; push or poll? | `read()` timing | `udphandler.cpp` |

## 2. Commands

| Fact | Needed for | Notes |
|---|---|---|
| CSP target write: which CANopen object/index, counts scaling | `sendPositionTarget` | `epos.cpp` wraps SDO/PDO writes |
| PVM target write: object/index, unit (rpm vs counts/s) | `sendVelocityTarget` | |
| Mode switch sequence (disable → set mode → enable) and its timing | `setOperationMode` | `robot.h ActiveCSP(id)` is the existing example for CSP |
| Quick-stop control-word sequence | `quickStopAll` | |
| Whether velocity mode is actually used on the real robot today | validation | current code paths mostly use CSP |

## 3. Feedback scaling

| Fact | Needed for | Notes |
|---|---|---|
| Encoder resolution (counts/rev) at joint vs motor side | counts → rad | template uses 4096 as an EXAMPLE, verify |
| Homing offsets per joint | zero-pose alignment | `robot.h offset[13]` (body), hand equivalents unknown |
| Direction/sign per joint | URDF sign convention | `robot.h ratio[13]` / `Direction[13]` |
| Gear reduction per joint | counts → rad | `SimpleTransmission mechanicalReduction=1` in URDF is a placeholder |
| Torque estimate source (current-based? none?) | `effort` field | fingers: effort=1.0 in URDF is a placeholder |

## 4. Node-ID map

The mapping **joint name → EPOS node ID** for all 13 body actuators and the
12 hand actuators (6 per hand), plus which URDF joints are driven by each
hand actuator (the fingers are coupled; `gazebo_bridge.cpp` documents the
6-actuator → 22-joint coupling used in simulation).

## 5. Safety

| Fact | Needed for |
|---|---|
| Real continuous/peak torque limits per joint group | replacing the URDF `effort=1000` placeholder in `config/joint_limits.yaml` |
| Real maximum joint speeds | replacing the URDF `velocity=0.5` placeholder |
| Watchdog behaviour of the bridge (what happens on link loss) | `kCommandTimeout`, failure handling |
| Homing procedure requirements at startup | safe initial conditions |

Until these are filled in, the template must stay out of the build
(CMakeLists keeps it commented) and **no physical-hardware claim should be
made** for this package.
