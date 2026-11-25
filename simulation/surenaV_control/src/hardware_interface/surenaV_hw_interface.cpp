// =============================================================================
// surenaV_hw_interface.cpp -- see surenaV_hw_interface.h for the contract.
// TEMPLATE: the actuator link (EposDriverStub) is not implemented yet; every
// TODO(HW) marks a fact that must come from the real EPOS4/UDP bridge or the
// existing surena_eth package.
// =============================================================================
#include "surenaV_hw_interface.h"

#include <algorithm>
#include <chrono>
#include <thread>

namespace surenaV_control
{

bool SurenaVHardwareInterface::init(ros::NodeHandle& root_nh,
                                    ros::NodeHandle& robot_hw_nh)
{
  nh_ = robot_hw_nh;

  if (!loadJointConfiguration(robot_hw_nh))
  {
    ROS_FATAL("SurenaVHardwareInterface: no joint configuration found "
              "(expected ~joints list with per-joint epos_node_id).");
    return false;
  }

  registerInterfaces(robot_hw_nh);

  // Start in the safe state: motors detached, zero commands.
  if (!switchControlMode(ControlMode::IDLE))
    return false;

  last_successful_read_ = ros::Time::now();
  last_command_time_ = ros::Time(0);
  ROS_INFO("SurenaVHardwareInterface initialised with %zu joints.",
           joints_.size());
  return true;
}

bool SurenaVHardwareInterface::loadJointConfiguration(ros::NodeHandle& robot_hw_nh)
{
  XmlRpc::XmlRpcValue joint_list;
  if (!robot_hw_nh.getParam("joints", joint_list) ||
      joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  for (int i = 0; i < joint_list.size(); ++i)
  {
    JointData jd;
    jd.name = static_cast<std::string>(joint_list[i]["name"]);
    jd.epos_node_id = static_cast<int>(joint_list[i]["epos_node_id"]);
    if (joint_list[i].hasMember("direction"))
      jd.direction = static_cast<double>(joint_list[i]["direction"]);
    if (joint_list[i].hasMember("reduction"))
      jd.reduction = static_cast<double>(joint_list[i]["reduction"]);
    if (joint_list[i].hasMember("encoder_offset_counts"))
      jd.encoder_offset_counts =
          static_cast<double>(joint_list[i]["encoder_offset_counts"]);
    joints_.push_back(jd);
  }
  return !joints_.empty();
}

void SurenaVHardwareInterface::registerInterfaces(ros::NodeHandle& robot_hw_nh)
{
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_WARN("SurenaVHardwareInterface: no robot_description on the parameter "
             "server; joint limits will come from rosparam only.");

  for (auto& jd : joints_)
  {
    // 1) Joint state interface: every joint reports position/velocity/effort.
    hardware_interface::JointStateHandle js_handle(
        jd.name, &jd.position, &jd.velocity, &jd.effort);
    js_interface_.registerHandle(js_handle);

    // 2) Command interfaces: BOTH are registered; the active ControlMode
    //    decides which command buffer write() transmits. Controllers for the
    //    inactive interface must not be running (enforced by the documented
    //    switch sequence: stop controllers -> switchControlMode -> start).
    hardware_interface::JointHandle pos_handle(
        js_interface_.getHandle(jd.name), &jd.position_cmd);
    pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(
        js_interface_.getHandle(jd.name), &jd.velocity_cmd);
    vel_interface_.registerHandle(vel_handle);

    // 3) Soft limits from the URDF + rosparam (joint_limits/<joint>/...).
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(urdf_model.getJoint(jd.name), limits);
    joint_limits_interface::getJointLimits(robot_hw_nh, jd.name, limits);

    joint_limits_interface::PositionJointSoftLimitsHandle pos_limits(
        pos_handle, limits, joint_limits_interface::SoftJointLimits());
    pos_limits_interface_.registerHandle(pos_limits);

    joint_limits_interface::VelocityJointSoftLimitsHandle vel_limits(
        vel_handle, limits, joint_limits_interface::SoftJointLimits());
    vel_limits_interface_.registerHandle(vel_limits);
  }

  registerInterface(&js_interface_);
  registerInterface(&pos_interface_);
  registerInterface(&vel_interface_);
  registerInterface(&pos_limits_interface_);
  registerInterface(&vel_limits_interface_);
}

void SurenaVHardwareInterface::read(const ros::Time& time,
                                    const ros::Duration& /*period*/)
{
  // TODO(HW): replace with the real UDP/CAN feedback poll. surena_eth's
  // Robot::FeedBackReceived(...) shows the packet contents available from the
  // bridge (absolute/incremental positions, F/T, IMU, bump sensors).
  for (auto& jd : joints_)
  {
    EposFeedback fb;
    if (!EposDriverStub::readJoint(jd.epos_node_id, &fb))
    {
      ++consecutive_failures_;
      ROS_WARN_THROTTLE(1.0, "read() failed for %s (node %d), %d consecutive "
                             "failures", jd.name.c_str(), jd.epos_node_id,
                             consecutive_failures_);
      if (consecutive_failures_ >= kMaxConsecutiveFailures)
      {
        ROS_FATAL("Communication lost for %d consecutive reads -> emergency "
                  "stop. Recover the link, then re-activate controllers.",
                  consecutive_failures_);
        emergencyStop();
      }
      return;  // keep last known state; do not publish garbage
    }
    // TODO(HW): confirm scaling. Counts -> rad including direction and the
    // homing offset (surena_eth robot.h keeps `offset[]`/`ratio[]` tables).
    jd.position = jd.direction *
        (fb.position_counts - jd.encoder_offset_counts) *
        (2.0 * M_PI) / (4096.0 * jd.reduction);  // EXAMPLE scaling, verify!
    jd.velocity = jd.direction * fb.velocity_counts_to_rad_s;
    jd.effort = fb.torque_estimate_nm;
  }
  consecutive_failures_ = 0;
  last_successful_read_ = time;
}

void SurenaVHardwareInterface::write(const ros::Time& time,
                                     const ros::Duration& period)
{
  // Stale-command guard: if no controller refreshed its command recently,
  // hold the last known-good state instead of chasing a dead setpoint.
  const bool commands_stale =
      (time - last_command_time_).toSec() > kCommandTimeout;

  enforceLimits(period);

  for (auto& jd : joints_)
  {
    switch (mode_)
    {
      case ControlMode::POSITION:
      {
        const double cmd = commands_stale ? jd.last_known_good_position_cmd
                                          : jd.position_cmd;
        // No PID here: the EPOS4 runs its internal position loop (CSP).
        // TODO(HW): EposDriverStub::sendPositionTarget(node, counts(cmd)).
        EposDriverStub::sendPositionTarget(jd.epos_node_id, cmd);
        if (!commands_stale) jd.last_known_good_position_cmd = cmd;
        break;
      }
      case ControlMode::VELOCITY:
      {
        const double cmd = commands_stale ? jd.last_known_good_velocity_cmd
                                          : jd.velocity_cmd;
        // EPOS4 internal velocity loop (PVM). No PID here either.
        // TODO(HW): EposDriverStub::sendVelocityTarget(node, cmd).
        EposDriverStub::sendVelocityTarget(jd.epos_node_id, cmd);
        if (!commands_stale) jd.last_known_good_velocity_cmd = cmd;
        break;
      }
      case ControlMode::IDLE:
      default:
        // Safe state: zero-velocity command with the drive enabled, so the
        // joints hold instead of collapsing.
        // TODO(HW): EposDriverStub::sendVelocityTarget(node, 0.0).
        EposDriverStub::sendVelocityTarget(jd.epos_node_id, 0.0);
        break;
    }
  }
  if (!commands_stale) last_command_time_ = time;
}

bool SurenaVHardwareInterface::switchControlMode(ControlMode mode)
{
  if (mode == mode_)
    return true;

  // TODO(HW): EPOS4 mode switch is an SDO write per node (surena_eth's
  // epos.cpp wraps it; ActiveCSP(...) in robot.h is the existing example for
  // CSP). Sequence per actuator: disable operation -> set mode -> enable.
  if (!EposDriverStub::setOperationMode(mode == ControlMode::POSITION
                                            ? EposMode::CSP
                                            : EposMode::PVM))
  {
    ROS_ERROR("Actuators refused the mode switch; staying in mode %d.",
              static_cast<int>(mode_));
    return false;
  }
  mode_ = mode;
  ROS_INFO("Control mode switched to %s.",
           mode == ControlMode::POSITION ? "POSITION (EPOS4 CSP)"
                                         : mode == ControlMode::VELOCITY
                                               ? "VELOCITY (EPOS4 PVM)"
                                               : "IDLE");
  return true;
}

void SurenaVHardwareInterface::emergencyStop()
{
  // TODO(HW): quick-stop via the EPOS4 control word (surena_eth implements
  // the equivalent for its own fault handling).
  EposDriverStub::quickStopAll();
  mode_ = ControlMode::IDLE;
}

void SurenaVHardwareInterface::enforceLimits(const ros::Duration& period)
{
  pos_limits_interface_.enforceLimits(period);
  vel_limits_interface_.enforceLimits(period);
}

}  // namespace surenaV_control

// PLUGINLIB_EXPORT_CLASS for a combined gazebo/hardware build is not needed:
// this class is loaded directly by a custom controller_manager node
// (see README, "Real robot" section).
