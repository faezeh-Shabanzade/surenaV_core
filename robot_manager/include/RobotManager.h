#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "ros/ros.h"
#include <memory>
#include <map>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include "ros/callback_queue.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

// Include managers from library packages
#include "HandManager.h"
#include "GaitManager.h"

// Include our new service definition
#include "robot_manager/ExecuteScenario.h"
#include <robot_manager/JointCommand.h>
#include "surena_msgs/JointSensorRead.h"
#include "surena_msgs/SetJointAngle.h"

class RobotManager {
public:
    RobotManager(ros::NodeHandle *n);

private:
    bool simulation;

    // --- ROS Communication ---
    ros::NodeHandle* nh_;
    ros::ServiceServer execute_scenario_service_;
    ros::Subscriber publish_trigger_sub_;
    ros::ServiceServer joint_command_service_;
    ros::ServiceServer joints_sensor_read_service_;
    ros::ServiceServer set_joint_angle_service_;
    ros::ServiceClient hand_joints_sensor_read_client_;
    ros::ServiceClient gait_joints_sensor_read_client_;

    ros::Publisher combined_motor_pub_;
    ros::Publisher combined_gazebo_pub_;
    // Finger joint commands for Gazebo (22 values: right hand 0-10, left hand 11-21).
    ros::Publisher finger_gazebo_pub_;

    // Cache of per-joint command publishers used by /set_joint_angle in simulation.
    std::map<std::string, ros::Publisher> joint_pub_cache_;
    std::mutex joint_pub_mutex_;

    // --- Pointers to Specialized Managers ---
    std::unique_ptr<HandManager> hand_manager_;
    std::unique_ptr<GaitManager> gait_manager_;

    // message objects 
    std_msgs::Int32MultiArray combined_motor_command_msg_;
    std_msgs::Float64MultiArray combined_gazebo_command_msg_;

    // --- Member to hold the parsed scenario data ---
    YAML::Node scenarios_config_;

    // --- Service Handlers ---
    bool execute_scenario_callback(robot_manager::ExecuteScenario::Request &req, robot_manager::ExecuteScenario::Response &res);
    bool jointCommandCallback(robot_manager::JointCommand::Request &req, robot_manager::JointCommand::Response &res); 
    // --- Joint Sensor Read Client ---
    // One request type, two servers: hand_planner and gait_planner.
    bool readJointSensor(int32_t id, double &angle);
    // Single entry point for callers: dispatches to whichever planner owns the id.
    bool jointsSensorReadCallback(surena_msgs::JointSensorRead::Request &req,
                                  surena_msgs::JointSensorRead::Response &res);
    // Direct single-joint angle command (body + actuated finger joints, simulation only).
    bool setJointAngleCallback(surena_msgs::SetJointAngle::Request &req,
                               surena_msgs::SetJointAngle::Response &res);
    // Publish one joint position command to Gazebo (publisher cached per joint).
    void publishJointCommand(const std::string& joint_name, double angle_rad);

    // --- Helper Function ---
    void load_scenarios_from_file();
    bool execute_step(const YAML::Node& step);
    void publishTriggerCallback(const std_msgs::Empty::ConstPtr& msg);
    void publishCombinedMotorCommands();
    int32_t map_range(double val, double min_val, double max_val, double min_cmd, double max_cmd);
};

#endif // ROBOT_MANAGER_H