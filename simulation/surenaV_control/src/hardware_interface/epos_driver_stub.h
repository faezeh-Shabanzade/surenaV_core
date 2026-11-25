// =============================================================================
// epos_driver_stub.h
//
// HARDWARE-DRIVER ABSTRACTION -- deliberately NOT an implementation.
//
// The physical Surena V actuators are Maxon EPOS4 controllers behind a
// UDP-to-CAN bridge. The existing `surena_eth` package (src/epos.cpp,
// src/can.cpp, src/udphandler.cpp, src/robot.cpp) already speaks that
// protocol for the Qt-based teleop stack, but it is not usable as a library
// from a ros_control RobotHW as-is.
//
// RULE (from the project requirements): do not invent the communication
// protocol. This stub defines the exact seam where the real driver plugs in,
// and the companion HARDWARE_INFO_NEEDED.md lists every fact required to
// implement it. Until then, linking fails with a clear "not implemented"
// message instead of silently doing nothing.
// =============================================================================
#ifndef EPOS_DRIVER_STUB_H
#define EPOS_DRIVER_STUB_H

#include <cstdint>

namespace surenaV_control
{

/// EPOS4 operation modes relevant to this project (numbers match the
/// `EPOSOperationMode` enum in surena_eth/src/epos.h).
enum class EposMode : uint8_t
{
  PPM = 1,   ///< profile position
  PVM = 3,   ///< profile velocity
  HMM = 6,   ///< homing
  CSP = 8,   ///< cyclic synchronous position  <- position control
  CST = 10,  ///< cyclic synchronous torque
};

/// Feedback packet for one joint, already unpacked from the bridge frame.
/// TODO(HW): confirm field list against the real feedback frame
/// (surena_eth Robot::FeedBackReceived suggests: absolute position,
/// incremental position, F/T, IMU, bump sensors, hall effect).
struct EposFeedback
{
  int32_t position_counts = 0;      ///< absolute encoder counts
  double velocity_counts_to_rad_s = 0.0;  ///< velocity in rad/s at the joint
  double torque_estimate_nm = 0.0;  ///< estimated or measured torque
  bool fault = false;               ///< controller fault flag
};

/// Static seam for the real driver. Every method is a TODO(HW).
class EposDriverStub
{
public:
  EposDriverStub() = delete;

  /// Poll one joint's feedback. False on communication failure.
  static bool readJoint(int node_id, EposFeedback* out);

  /// Position target in radians at the JOINT side; the driver converts to
  /// counts using the calibration tables (direction, reduction, offset).
  static bool sendPositionTarget(int node_id, double target_rad);

  /// Velocity target in rad/s at the JOINT side.
  static bool sendVelocityTarget(int node_id, double target_rad_s);

  /// Switch the operation mode of ALL nodes (disable -> set mode -> enable).
  static bool setOperationMode(EposMode mode);

  /// EPOS4 quick-stop on every node (control word quick-stop sequence).
  static void quickStopAll();
};

}  // namespace surenaV_control

#endif  // EPOS_DRIVER_STUB_H
