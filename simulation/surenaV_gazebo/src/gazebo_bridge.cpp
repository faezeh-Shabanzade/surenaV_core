#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <string>

// Global Publishers
std::vector<ros::Publisher> right_leg_pubs;
std::vector<ros::Publisher> left_leg_pubs;
std::vector<ros::Publisher> right_arm_pubs;
std::vector<ros::Publisher> left_arm_pubs;
std::vector<ros::Publisher> head_pubs;
std::vector<ros::Publisher> right_finger_pubs;
std::vector<ros::Publisher> left_finger_pubs;

// Finger joint layout (11 per hand), matching HandManager::finger_gazebo_commands_:
//   [middle_finger1..4, lower_thumb, middle_thumb, upper_finger1..4, upper_thumb]
// The upper (distal) joints are <mimic> joints in the URDF; Gazebo classic does not
// enforce mimic tags, so the bridge re-applies the coupling here as well.
static const std::vector<std::string> kFingerJoints = {
    "middle_finger1", "middle_finger2", "middle_finger3", "middle_finger4",
    "lower_thumb", "middle_thumb",
    "upper_finger1", "upper_finger2", "upper_finger3", "upper_finger4",
    "upper_thumb"
};
// multiplier applied to the driving joint angle for each mimic joint (index 6..10),
// derived from the PRISTINE SolidWorks URDF (surenaV_hands.urdf): with that model's
// joint frames the upper axis equals the middle axis on both hands, so every
// multiplier is +1. If a distal joint ever moves the wrong way, flip the entry here.
static const std::vector<double> kMimicMultiplier  = { 1.0, 1.0, 1.0, 1.0, 1.0}; // right hand
static const std::vector<double> kMimicMultiplierL = { 1.0, 1.0, 1.0, 1.0, 1.0}; // left hand
// which actuated joint (index into the first 6 entries) drives each mimic joint
static const std::vector<int> kMimicSource = {0, 1, 2, 3, 5};

// callback for whole body
void GazeboCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // Right Leg
    for (int i = 0; i < 6; ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[i];
        right_leg_pubs[i].publish(cmd);
    }
    // Left Leg
    for (int i = 0; i < 6; ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[i + 6];
        left_leg_pubs[i].publish(cmd);
    }
    // Right Arm
    std::vector<int> right_arm_indices = {12, 13, 14, 15, 23, 24, 25};
    for (size_t i = 0; i < right_arm_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[right_arm_indices[i]];
        right_arm_pubs[i].publish(cmd);
    }
    // Left Arm
    std::vector<int> left_arm_indices = {16, 17, 18, 19, 26, 27, 28};
    for (size_t i = 0; i < left_arm_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[left_arm_indices[i]];
        left_arm_pubs[i].publish(cmd);
    }
    // Head
    std::vector<int> head_indices = {20, 21, 22};
    for (size_t i = 0; i < head_pubs.size(); ++i) {
        std_msgs::Float64 cmd;
        cmd.data = msg->data[head_indices[i]];
        head_pubs[i].publish(cmd);
    }
}

// callback for finger joint angles (22 values: right hand 0-10, left hand 11-21).
// Mimic coupling is re-applied here so the URDF <mimic> relations hold in Gazebo
// even if the upstream node only commands the 6 actuated joints per hand.
void FingerGazeboCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 22) {
        ROS_WARN_THROTTLE(5.0, "/finger_angles_gazebo expects 22 values, got %zu", msg->data.size());
        return;
    }
    double hands[2][11];
    for (int h = 0; h < 2; ++h) {
        const std::vector<double>& mimic_mult = (h == 0) ? kMimicMultiplier : kMimicMultiplierL;
        for (int i = 0; i < 11; ++i)
            hands[h][i] = msg->data[h * 11 + i];
        // enforce mimic coupling on the distal joints (indices 6..10)
        for (int m = 0; m < 5; ++m)
            hands[h][6 + m] = mimic_mult[m] * hands[h][kMimicSource[m]];
    }
    for (int i = 0; i < 11; ++i) {
        std_msgs::Float64 cmd;
        cmd.data = hands[0][i];
        right_finger_pubs[i].publish(cmd);
        cmd.data = hands[1][i];
        left_finger_pubs[i].publish(cmd);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_bridge_node");
    ros::NodeHandle nh;

    // Right Leg Publishers
    std::vector<std::string> right_leg_topics = {
        "/surenaV/r_hip_yaw_joint_position_controller/command",
        "/surenaV/r_hip_roll_joint_position_controller/command",
        "/surenaV/r_hip_pitch_joint_position_controller/command",
        "/surenaV/r_knee_joint_position_controller/command",
        "/surenaV/r_ankle_pitch_joint_position_controller/command",
        "/surenaV/r_ankle_roll_joint_position_controller/command"
    };
    for (const auto& topic : right_leg_topics) {
        right_leg_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Left Leg Publishers
    std::vector<std::string> left_leg_topics = {
        "/surenaV/l_hip_yaw_joint_position_controller/command",
        "/surenaV/l_hip_roll_joint_position_controller/command",
        "/surenaV/l_hip_pitch_joint_position_controller/command",
        "/surenaV/l_knee_joint_position_controller/command",
        "/surenaV/l_ankle_pitch_joint_position_controller/command",
        "/surenaV/l_ankle_roll_joint_position_controller/command"
    };
    for (const auto& topic : left_leg_topics) {
        left_leg_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Right Arm Publishers
    std::vector<std::string> right_arm_topics = {
        "/surenaV/r_arm_pitch_joint_position_controller/command",
        "/surenaV/r_arm_roll_joint_position_controller/command",
        "/surenaV/r_elbow_pitch_joint_position_controller/command",
        "/surenaV/r_forearm_roll_joint_position_controller/command",
        "/surenaV/r_forearm_link_joint_position_controller/command",
        "/surenaV/r_hand_roll_joint_position_controller/command",
        "/surenaV/r_hand_pitch_joint_position_controller/command"
    };
    for (const auto& topic : right_arm_topics) {
        right_arm_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Left Arm Publishers
    std::vector<std::string> left_arm_topics = {
        "/surenaV/l_arm_pitch_joint_position_controller/command",
        "/surenaV/l_arm_roll_joint_position_controller/command",
        "/surenaV/l_elbow_pitch_joint_position_controller/command",
        "/surenaV/l_forearm_roll_joint_position_controller/command",
        "/surenaV/l_forearm_link_joint_position_controller/command",
        "/surenaV/l_hand_roll_joint_position_controller/command",
        "/surenaV/l_hand_pitch_joint_position_controller/command"
    };
    for (const auto& topic : left_arm_topics) {
        left_arm_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Head Publishers
    std::vector<std::string> head_topics = {
        "/surenaV/head_roll_joint_position_controller/command",
        "/surenaV/head_pitch_joint_position_controller/command",
        "/surenaV/head_yaw_joint_position_controller/command"
    };
    for (const auto& topic : head_topics) {
        head_pubs.push_back(nh.advertise<std_msgs::Float64>(topic, 10));
    }

    // Finger Publishers (actuated + mimic joints, one position controller each)
    for (int h = 0; h < 2; ++h) {
        const std::string side = (h == 0) ? "r" : "l";
        for (const auto& j : kFingerJoints) {
            auto pub = nh.advertise<std_msgs::Float64>(
                "/surenaV/" + side + "_" + j + "_joint_position_controller/command", 10);
            if (h == 0)
                right_finger_pubs.push_back(pub);
            else
                left_finger_pubs.push_back(pub);
        }
    }

    // Subscribers
    ros::Subscriber joint_sub = nh.subscribe("/joint_angles_gazebo", 10, GazeboCallback);
    ros::Subscriber finger_sub = nh.subscribe("/finger_angles_gazebo", 10, FingerGazeboCallback);

    ROS_INFO("Surena Gazebo bridge node is running (body + fingers with mimic).");
    ros::spin();

    return 0;
}
