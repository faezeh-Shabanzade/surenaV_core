// =============================================================================
// surenaV_hw_interface.h
//
// TEMPLATE hardware_interface::RobotHW for the physical Surena V.
//
// Purpose: let the SAME ros_control controller configuration that runs in
// Gazebo/PyBullet run on the real robot. The actuators (Maxon EPOS4 behind a
// UDP/CAN bridge, see the existing surena_eth package) already implement
// internal position/velocity control loops, so this interface only forwards
// commands -- it deliberately contains NO second PID loop.
//
// STATUS: TEMPLATE. The EPOS4 communication layer is an abstraction
// (EposDriverStub, epos_driver_stub.h); fill it with the real protocol using
// surena_eth/src/{epos.cpp,can.cpp,udphandler.cpp} as the reference. The
// parts marked TODO(HW) need hardware-specific facts that are not available
// in this repository (see HARDWARE_INFO_NEEDED.md).
//
// Not compiled by default: uncomment the block in CMakeLists.txt once the
// driver layer is implemented.
// =============================================================================
#ifndef SURENAV_HW_INTERFACE_H
#define SURENAV_HW_INTERFACE_H

#include <string>
#include <vector>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "epos_driver_stub.h"

namespace surenaV_control
{

/// Which actuator internal mode the bridge is currently in. Mirrors the
/// EPOS4 operation modes used by surena_eth (PVM=3 profile velocity,
/// CSP=8 cyclic synchronous position).
enum class ControlMode : uint8_t
{
  IDLE = 0,      ///< motors detached / safe
  POSITION = 1,  ///< EPOS4 CSP (cyclic synchronous position)
  VELOCITY = 2,  ///< EPOS4 PVM (profile velocity)
};

class SurenaVHardwareInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  /// Safe mode switch. Call ONLY while no controller is running
  /// (controller_manager/switch_controller with empty start list first).
  /// Returns false (and keeps the old mode) if the actuators refuse.
  bool switchControlMode(ControlMode mode);

  /// Immediate safe state: zero velocity commands, position commands held.
  void emergencyStop();

private:
  /// One joint's runtime state. Arrays-of-structs keeps the EPOS node-id
  /// mapping next to the data it belongs to.
  struct JointData
  {
    std::string name;
    int epos_node_id = -1;      ///< actuator node on the CAN/UDP bridge
    double position = 0.0;      ///< feedback (rad, URDF direction)
    double velocity = 0.0;      ///< feedback (rad/s)
    double effort = 0.0;        ///< feedback (N.m, estimated or measured)
    double position_cmd = 0.0;  ///< from PositionJointInterface
    double velocity_cmd = 0.0;  ///< from VelocityJointInterface
    double last_known_good_position_cmd = 0.0;
    double last_known_good_velocity_cmd = 0.0;
    // actuator-side scaling (TODO(HW): confirm from surena_eth calibration)
    double encoder_offset_counts = 0.0;   ///< home offset (see robot.h `offset[]`)
    double direction = 1.0;               ///< URDF sign vs actuator sign
    double reduction = 1.0;               ///< gearbox + SimpleTransmission ratio
  };

  void registerInterfaces(ros::NodeHandle& robot_hw_nh);
  bool loadJointConfiguration(ros::NodeHandle& robot_hw_nh);
  void enforceLimits(const ros::Duration& period);

  ros::NodeHandle nh_;
  std::vector<JointData> joints_;

  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pos_interface_;
  hardware_interface::VelocityJointInterface vel_interface_;

  joint_limits_interface::PositionJointSoftLimitsInterface pos_limits_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface_;

  ControlMode mode_ = ControlMode::IDLE;

  /// Communication watchdog: how long since the last successful read().
  ros::Time last_successful_read_;
  int consecutive_failures_ = 0;
  static constexpr int kMaxConsecutiveFailures = 10;
  static constexpr double kCommandTimeout = 0.5;  ///< s; stale command -> hold
  ros::Time last_command_time_;
};

}  // namespace surenaV_control

#endif  // SURENAV_HW_INTERFACE_H
