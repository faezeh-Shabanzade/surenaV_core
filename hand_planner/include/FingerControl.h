#pragma once
#ifndef FINGER_CONTROL_H
#define FINGER_CONTROL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <mutex>
#include "json.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>

using namespace std;
using json = nlohmann::json;

// Finger Motor Data Structure: 29 + (0-5: target positions, 6-7: controls)
// Servo positions are 0-255 per channel (255 = fully open, 0 = fully closed).
// 6 servo channels per hand, in this order:
//   [finger1, finger2, finger3, finger4, thumb_lower (opposition), thumb_middle (flex)]

// Gazebo finger joint order used by finger_gazebo_commands_ (11 per hand, right first):
//   [middle_finger1..4, lower_thumb, middle_thumb, upper_finger1..4, upper_thumb]
// The first 6 are actuated (the hardware servo channels); the last 5 are the URDF
// <mimic> joints and are computed from their driving joint with the multipliers below.
static const std::vector<std::string> kFingerGazeboJointNames = {
    "middle_finger1", "middle_finger2", "middle_finger3", "middle_finger4",
    "lower_thumb", "middle_thumb",
    "upper_finger1", "upper_finger2", "upper_finger3", "upper_finger4",
    "upper_thumb"
};
// Mimic coupling between the distal (upper) joints and their driving joint,
// derived from the PRISTINE SolidWorks URDF (surenaV_hands.urdf): with that
// model's joint frames the upper axis equals the middle axis on BOTH hands, so
// every multiplier is +1 (distal angle = driving angle).
static const std::vector<double> kMimicMultiplier    = { 1.0,  1.0, 1.0, 1.0, 1.0}; // right hand
static const std::vector<double> kMimicMultiplierL   = { 1.0,  1.0, 1.0, 1.0, 1.0}; // left hand
static const std::vector<int>    kMimicSourceIdx  = {0, 1, 2, 3, 5};   // actuated index driving mimic 6..10

// Per-channel sign applied to the GAZEBO joint command only (hardware servo
// values are unaffected), in channel order [finger1..4, lower_thumb, middle_thumb].
// finger1..4 and middle_thumb are +1: with the pristine SolidWorks URDF a
// positive flex angle curls them toward the palm on both hands.
// FIX (lower_thumb moves backward): the lower_thumb axes in the URDF are
// handed (r_lower_thumb axis z<0, l_lower_thumb axis z>0), so with the shared
// flex-angle convention the Gazebo lower_thumb joints rotated the opposite
// way to the hardware servos on BOTH hands. kGazeboSignR/L[4] = -1 negates
// the Gazebo lower_thumb command only; /set_joint_angle reads the same
// tables, so both command paths are corrected by this single change.
static const std::vector<double> kGazeboSignR = { 1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
static const std::vector<double> kGazeboSignL = { 1.0, 1.0, 1.0, 1.0, -1.0, 1.0};

// Servo <-> angle mapping (tunable; see finger_scenarios.json "servo_mapping")
static const int    kServoOpen   = 255;   // servo value at fully open hand
static const int    kServoClosed = 0;     // servo value at fully closed hand
static const double kMaxFlexRad  = 1.5;   // joint angle [rad] at fully closed

// Hand selection enum
enum class HandSelection {
    RIGHT_HAND = 0,
    LEFT_HAND = 1,
};

struct FingerScenario {
    std::string name;
    std::vector<uint8_t> target_positions;   // 0-255
    uint8_t control_data;
    // Timed (byebye-style) scenarios: keyframes of servo positions; t is the time
    // [s] within one cycle. If empty the scenario is a static pose.
    std::vector<std::pair<double, std::vector<uint8_t>>> keyframes;

    FingerScenario() : control_data(0) {
        target_positions.resize(6, 0);
    }
};

class FingerControl {
public:
    FingerControl(ros::NodeHandle* nh);

    // Scenario management
    bool executeScenario(const std::string& name, HandSelection hand = HandSelection::RIGHT_HAND);

    // Timed (byebye-style) animation: blocking version, runs at 200 Hz for
    // duration * repetitions seconds and publishes through the trigger.
    bool executeAnimation(const std::string& name, HandSelection hand,
                          double duration_per_cycle, int repetitions);
    // Non-blocking version: startAnimation() arms the animation, tickAnimation()
    // advances it (call from an existing control loop, e.g. the arm trajectory),
    // and stopAnimation() finalizes it.
    bool startAnimation(const std::string& name, HandSelection hand,
                        double duration_per_cycle, int repetitions);
    // Convenience for arm scenarios: run the scenario so that it spans exactly
    // total_time seconds. Keyframed scenarios repeat over their natural cycle
    // (last keyframe time); static poses simply hold for total_time. When the
    // animation ends, tickAnimation() holds the final gesture pose.
    bool startAnimationForDuration(const std::string& name, HandSelection hand,
                                   double total_time);
    bool tickAnimation();
    void stopAnimation();
    bool animationActive() const { return anim_active_; }

    // Direct control
    bool setDirectControl(const std::vector<uint8_t>& target_positions,
                          uint8_t control_data,
                          HandSelection hand = HandSelection::RIGHT_HAND);

    // Utility method to convert string to HandSelection
    static HandSelection stringToHandSelection(const std::string& hand_str);

    const std::vector<double>& getFingerCommands() const;
    // Full 22-value finger joint angle vector [rad] for Gazebo:
    // right hand 0-10, left hand 11-21 (see kFingerGazeboJointNames).
    const std::vector<double>& getFingerGazeboCommands() const;
    // Last commanded servo positions per hand (for incremental direct control).
    const std::vector<uint8_t>& getLastServoPositions(HandSelection hand) const;

private:
    ros::NodeHandle* nh_;
    ros::Publisher publish_trigger_pub_;

    void applyServoPositions(const std::vector<uint8_t>& pos, HandSelection hand);
    void loadScenarios();

    std::map<std::string, FingerScenario> scenarios_;
    std::mutex scenarios_mutex_;

    vector<double> finger_commands_ = std::vector<double>(8, 0.0);
    std::vector<double> finger_gazebo_commands_ = std::vector<double>(22, 0.0);
    std::vector<uint8_t> last_positions_[2];

    // active animation state
    bool anim_active_ = false;
    FingerScenario anim_scenario_;
    HandSelection anim_hand_ = HandSelection::RIGHT_HAND;
    double anim_duration_ = 0.0;      // duration of one keyframe cycle [s]
    int anim_repetitions_ = 1;
    std::chrono::steady_clock::time_point anim_start_;
    double anim_total_ = 0.0;
};

#endif // FINGER_CONTROL_H
