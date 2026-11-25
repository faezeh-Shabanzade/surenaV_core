#include "FingerControl.h"

// Finger Motor Data Structure: 29 + (0-5: target positions, 6-7: controls)
// 0: close , 255: open

FingerControl::FingerControl(ros::NodeHandle* nh) : nh_(nh) {
    // Initialize ROS communication
    publish_trigger_pub_ = nh->advertise<std_msgs::Empty>("robot_manager/publish_trigger", 1);

    for (int h = 0; h < 2; ++h)
        last_positions_[h] = std::vector<uint8_t>(6, kServoOpen);

    // Start every finger joint fully open (0 rad)
    finger_gazebo_commands_.assign(22, 0.0);

    loadScenarios();
}

void FingerControl::loadScenarios() {
    // Load scenarios from JSON file
    std::string scenarios_path = ros::package::getPath("hand_planner") + "/config/finger_scenarios.json";
    std::ifstream fr(scenarios_path);
    json scenarios_json = json::parse(fr);

    scenarios_.clear();
    for (auto& [name, scenario_data] : scenarios_json["scenarios"].items()) {
        FingerScenario scenario;
        scenario.name = name;
        scenario.target_positions = scenario_data["target_positions"].get<std::vector<uint8_t>>();
        scenario.control_data = scenario_data["control_data"].get<uint8_t>();
        // Optional timed keyframes: {"t": seconds, "positions": [6 servo values]}
        if (scenario_data.contains("keyframes")) {
            for (auto& kf : scenario_data["keyframes"]) {
                double t = kf["t"].get<double>();
                std::vector<uint8_t> pos = kf["positions"].get<std::vector<uint8_t>>();
                scenario.keyframes.push_back({t, pos});
            }
            std::sort(scenario.keyframes.begin(), scenario.keyframes.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
        }
        scenarios_[name] = scenario;
    }
    ROS_INFO("FingerControl: loaded %zu scenarios", scenarios_.size());
}

// Convert one servo channel value (0-255) to a joint angle [rad].
// 255 (open) -> 0 rad, 0 (closed) -> kMaxFlexRad.
static inline double servoToAngle(uint8_t servo) {
    double flex = (double)(kServoOpen - (int)servo) / (double)(kServoOpen - kServoClosed);
    flex = std::max(0.0, std::min(1.0, flex));
    return flex * kMaxFlexRad;
}

// Update the hardware servo command vector and the 22-value Gazebo joint angle
// vector (including the mimic joints) from 6 servo channel positions of one hand.
void FingerControl::applyServoPositions(const std::vector<uint8_t>& pos, HandSelection hand) {
    const int h = (hand == HandSelection::RIGHT_HAND) ? 0 : 1;
    last_positions_[h] = pos;

    // Hardware servo command vector (0-5 positions, 6-7 controls)
    for (int i = 0; i < 8; ++i)
        finger_commands_[i] = 0.0;
    for (int i = 0; i < 6; ++i)
        finger_commands_[i] = pos[i];
    finger_commands_[6 + h] = 1;  // control byte selects the hand

    // Gazebo joint angles for this hand
    const int base = h * 11;
    const auto& gazebo_sign = (h == 0) ? kGazeboSignR : kGazeboSignL;
    for (int i = 0; i < 6; ++i)                     // actuated joints (sign-tunable)
        finger_gazebo_commands_[base + i] = gazebo_sign[i] * servoToAngle(pos[i]);
    for (int m = 0; m < 5; ++m) {                   // mimic joints (per-hand multipliers)
        const auto& table = (h == 0) ? kMimicMultiplier : kMimicMultiplierL;
        finger_gazebo_commands_[base + 6 + m] =
            table[m] * finger_gazebo_commands_[base + kMimicSourceIdx[m]];
    }
}

bool FingerControl::executeScenario(const std::string& name, HandSelection hand) {
    std::lock_guard<std::mutex> lock(scenarios_mutex_);
    auto it = scenarios_.find(name);
    if (it == scenarios_.end()) {
        ROS_ERROR("Scenario '%s' not found", name.c_str());
        return false;
    }
    const FingerScenario& scenario = it->second;
    if (!scenario.keyframes.empty()) {
        // keyframe scenario executed as a static pose: hold the last keyframe
        applyServoPositions(scenario.keyframes.back().second, hand);
    } else {
        applyServoPositions(scenario.target_positions, hand);
    }
    // Publish 2 messages
    ros::Rate rate(200);
    for (int i = 0; i < 2; i++) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
    }
    return true;
}

bool FingerControl::setDirectControl(const std::vector<uint8_t>& target_positions,
                                    uint8_t control_data,
                                    HandSelection hand) {
    // Validate input sizes
    if (target_positions.size() != 6) {
        ROS_ERROR("Invalid target positions size: %zu (expected: 6)", target_positions.size());
        return false;
    }
    applyServoPositions(target_positions, hand);
    // Publish 2 messages
    ros::Rate rate(200);
    for (int i = 0; i < 2; i++) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
    }
    return true;
}

// --- Timed (byebye-style) animations -----------------------------------------

bool FingerControl::startAnimation(const std::string& name, HandSelection hand,
                                   double duration_per_cycle, int repetitions) {
    std::lock_guard<std::mutex> lock(scenarios_mutex_);
    auto it = scenarios_.find(name);
    if (it == scenarios_.end()) {
        ROS_ERROR("Finger animation scenario '%s' not found", name.c_str());
        return false;
    }
    anim_scenario_ = it->second;
    anim_hand_ = hand;
    anim_duration_ = std::max(0.2, duration_per_cycle);
    anim_repetitions_ = std::max(1, repetitions);
    anim_total_ = anim_duration_ * anim_repetitions_;
    anim_start_ = std::chrono::steady_clock::now();
    anim_active_ = true;
    return true;
}

bool FingerControl::startAnimationForDuration(const std::string& name, HandSelection hand,
                                              double total_time) {
    // Static poses (no keyframes) simply hold for the whole duration.
    // Keyframed scenarios split in two:
    //  - hold-type (last keyframe pose == the scenario's gesture target, e.g.
    //    'point'): play ONE natural cycle, then tickAnimation() holds the
    //    gesture for the rest of the arm motion;
    //  - periodic (e.g. 'finger_wave', returns to open): repeat over the
    //    natural cycle until the arm motion ends.
    double cycle = 0.2;
    int reps = 1;
    {
        std::lock_guard<std::mutex> lock(scenarios_mutex_);
        auto it = scenarios_.find(name);
        if (it == scenarios_.end()) {
            ROS_ERROR("Finger animation scenario '%s' not found", name.c_str());
            return false;
        }
        const auto& kfs = it->second.keyframes;
        if (!kfs.empty()) {
            cycle = std::max(0.2, kfs.back().first);
            bool hold_type = (kfs.back().second == it->second.target_positions);
            reps = hold_type ? 1 : std::max(1, (int)std::ceil(total_time / cycle));
        } else {
            cycle = std::max(0.2, total_time);
        }
    }
    return startAnimation(name, hand, cycle, reps);
}

bool FingerControl::tickAnimation() {
    if (!anim_active_)
        return false;
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - anim_start_).count();
    if (t >= anim_total_) {
        // Hold the final gesture pose: apply the last keyframe (or the static
        // target) once, then deactivate. The position controllers keep the
        // last commanded pose, so the gesture persists until the arm scenario
        // ends or a new finger command arrives.
        const auto& kfs = anim_scenario_.keyframes;
        std::vector<uint8_t> last = kfs.empty() ? anim_scenario_.target_positions
                                                : kfs.back().second;
        applyServoPositions(last, anim_hand_);
        anim_active_ = false;
        return false;
    }
    const auto& kfs = anim_scenario_.keyframes;
    if (kfs.empty()) {
        // static pose scenario used as animation: hold its target positions
        applyServoPositions(anim_scenario_.target_positions, anim_hand_);
        return true;
    }
    double t_cycle = std::fmod(t, anim_duration_);
    // find surrounding keyframes (cycle wraps from last back to first)
    size_t k = 0;
    while (k + 1 < kfs.size() && kfs[k + 1].first <= t_cycle)
        ++k;
    const auto& a = kfs[k];
    const auto& b = kfs[(k + 1) % kfs.size()];
    double span = ((k + 1 < kfs.size()) ? b.first : anim_duration_ + kfs.front().first) - a.first;
    double u = (span > 1e-9) ? (t_cycle - a.first) / span : 1.0;
    u = u * u * (3.0 - 2.0 * u);  // smoothstep for smooth starts/stops

    std::vector<uint8_t> pos(6);
    for (int i = 0; i < 6; ++i) {
        double v = (1.0 - u) * a.second[i] + u * b.second[i];
        pos[i] = (uint8_t)std::max(0, std::min(255, (int)std::lround(v)));
    }
    applyServoPositions(pos, anim_hand_);
    return true;
}

void FingerControl::stopAnimation() {
    anim_active_ = false;
}

bool FingerControl::executeAnimation(const std::string& name, HandSelection hand,
                                     double duration_per_cycle, int repetitions) {
    if (!startAnimation(name, hand, duration_per_cycle, repetitions))
        return false;
    ros::Rate rate(200);
    while (tickAnimation()) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
    }
    return true;
}

HandSelection FingerControl::stringToHandSelection(const std::string& hand_str) {
    // Convert to lowercase for case-insensitive comparison
    std::string lower_hand = hand_str;
    std::transform(lower_hand.begin(), lower_hand.end(), lower_hand.begin(), ::tolower);

    if (lower_hand == "right") {
        return HandSelection::RIGHT_HAND;
    } else if (lower_hand == "left") {
        return HandSelection::LEFT_HAND;
    } else {
        ROS_WARN("Invalid hand selection: '%s', defaulting to RIGHT_HAND", hand_str.c_str());
        return HandSelection::RIGHT_HAND;  // Default to right hand
    }
}

const std::vector<double>& FingerControl::getFingerCommands() const {
    return finger_commands_;
}

const std::vector<double>& FingerControl::getFingerGazeboCommands() const {
    return finger_gazebo_commands_;
}

const std::vector<uint8_t>& FingerControl::getLastServoPositions(HandSelection hand) const {
    return last_positions_[(hand == HandSelection::RIGHT_HAND) ? 0 : 1];
}
