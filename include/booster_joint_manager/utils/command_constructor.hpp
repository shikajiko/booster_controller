#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

inline constexpr float kDefaultJointDq = 0.0F;
inline constexpr float kDefaultJointTau = 0.0F;
inline constexpr float kDefaultJointWeight = 0.0F;
inline constexpr float kControlDt = 0.02F;
inline constexpr float kJointWeightRate = 0.2F;
inline constexpr float kBaseJointVelocity = 0.1F; //rad/s
inline constexpr float kMaxJointVelocity = 0.5F;  //rad/s
inline constexpr float kWeightMargin = kJointWeightRate * kControlDt;
inline constexpr float kBaseJointStep = kBaseJointVelocity * kControlDt;
inline constexpr float kMaxJointDelta = kMaxJointVelocity * kControlDt;
inline constexpr int kCommandFrequencyMs = static_cast<int>(kControlDt / 0.001F);

inline constexpr std::array<float, kJointCnt> kDefaultJointKps = {
  40., 40.,
  40., 50., 20., 20,
  40., 50., 20., 20,
  350., 350., 180., 350., 250., 250.,
  350., 350., 180., 350., 250., 250.
};

inline constexpr std::array<float, kJointCnt> kDefaultJointKds = {
  1.5, 1.5,
  0.5, 1.5, 0.2, 0.2,
  0.5, 1.5, 0.2, 0.2,
  7.5, 7.5, 3., 5.5, 5.0, 5.0,
  7.5, 7.5, 3., 5.5, 5.0, 5.0,
};

inline constexpr std::array<float, kJointCnt> kStandPose = {
    0, 0,
  0.0, -1.3, 0, -0.,
  0.0, 1.3, 0, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.
};

inline constexpr std::array<float, kJointCnt> kMinJointLimit = {
    -1.029744f,  // Head Yaw Joint (-59°)
    -0.331613f,  // Head Pitch Joint (-19°)
    -2.949606f,  // Left Shoulder Pitch Joint (-169°)
    -1.640609f,  // Left Shoulder Roll Joint (-94°)
    -1.902409f,  // Left Shoulder Yaw Joint (-109°)
    -2.251475f,  // Left Elbow Joint (-129°)
    -2.949606f,  // Right Shoulder Pitch Joint (-169°)
    -1.640609f,  // Right Shoulder Roll Joint (-94°)
    -1.902409f,  // Right Shoulder Yaw Joint (-109°)
    -0.680678f,  // Right Elbow Joint (-39°)
    -2.967060f,  // Left Hip Pitch Joint (-170°)
    -0.383972f,  // Left Hip Roll Joint (-22°)
    -1.029744f,  // Left Hip Yaw Joint (-59°)
     0.000000f,  // Left Knee Joint (0°)
    -0.296706f,  // Left Ankle Up Joint (-17°)
    -0.279253f,  // Left Ankle Down Joint (-16°)
    -2.967060f,  // Right Hip Pitch Joint (-170°)
    -1.553343f,  // Right Hip Roll Joint (-89°)
    -1.029744f,  // Right Hip Yaw Joint (-59°)
     0.000000f,  // Right Knee Joint (0°)
    -0.296706f,  // Right Ankle Up Joint (-17°)
    -0.279253f   // Right Ankle Down Joint (-16°)
};

inline constexpr std::array<float, kJointCnt> kMaxJointLimit = {
     1.029744f,  // Head Yaw Joint (59°)
     0.855211f,  // Head Pitch Joint (49°)
     1.204277f,  // Left Shoulder Pitch Joint (69°)
     1.640609f,  // Left Shoulder Roll Joint (94°)
     1.902409f,  // Left Shoulder Yaw Joint (109°)
     0.680678f,  // Left Elbow Joint (39°)
     1.204277f,  // Right Shoulder Pitch Joint (69°)
     1.640609f,  // Right Shoulder Roll Joint (94°)
     1.902409f,  // Right Shoulder Yaw Joint (109°)
     2.251475f,  // Right Elbow Joint (129°)
     2.234021f,  // Left Hip Pitch Joint (128°)
     1.553343f,  // Left Hip Roll Joint (89°)
     1.029744f,  // Left Hip Yaw Joint (59°)
     2.321288f,  // Left Knee Joint (133°)
     0.663225f,  // Left Ankle Up Joint (38°)
     0.715585f,  // Left Ankle Down Joint (41°)
     2.234021f,  // Right Hip Pitch Joint (128°)
     0.383972f,  // Right Hip Roll Joint (22°)
     1.029744f,  // Right Hip Yaw Joint (59°)
     2.321288f,  // Right Knee Joint (133°)
     0.663225f,  // Right Ankle Up Joint (38°)
     0.715585f   // Right Ankle Down Joint (41°)
};
struct JointCommandTarget
{
  JointIndex joint;
  float position;
  float velocity;
  float weight;
};

using MotorState = booster_interface::msg::MotorState;

booster_interface::msg::LowCmd construct_joint_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets);

booster_interface::msg::LowCmd construct_set_torque_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets,
  bool enable_torque);

}  // namespace booster_joint_manager
