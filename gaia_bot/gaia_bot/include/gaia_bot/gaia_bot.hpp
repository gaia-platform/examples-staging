// Copyright 2022 Gaia Platform, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAIA_BOT__GAIA_BOT_HPP_
#define GAIA_BOT__GAIA_BOT_HPP_

#include <gaia/system.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "gaia_gaia_bot.h"  // NOLINT

namespace gaia_bot
{

/// \class GaiaBot
/// \brief ROS 2 Node for GaiaBot behaviors.
class GaiaBot : public rclcpp::Node
{
public:
  /// \brief Default constructor, starts node.
  explicit GaiaBot(const rclcpp::NodeOptions & options);

  /// \brief Destructor, necessary to properly shut down Gaia.
  ~GaiaBot();

  /// \brief Utility function to publish a neck pose.
  static void publish_neck_pose(float rotation, float lift);

private:
  static constexpr std::chrono::milliseconds c_ego_time_update_interval = std::chrono::milliseconds(50);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub{};

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_battery_state_sub{};
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr m_left_light_sub{};
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr m_right_light_sub{};
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr m_range_sub{};
  rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr m_face_detection_sub{};

  static bool s_running;

  /// \brief Periodically update ego time.
  static void ego_time_update();

  /// \brief Handle incoming messages.
  void left_light_callback(const sensor_msgs::msg::Illuminance::ConstSharedPtr msg);

  /// \brief Handle incoming messages.
  void right_light_callback(const sensor_msgs::msg::Illuminance::ConstSharedPtr msg);

  /// \brief Handle incoming messages.
  void battery_state_callback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);

  /// \brief Handle incoming messages.
  void range_callback(const sensor_msgs::msg::Range::ConstSharedPtr msg);

  /// \brief Handle incoming messages.
  void face_detection_callback(const vision_msgs::msg::Detection3D::ConstSharedPtr msg);

  /// \brief Properly shutdown Gaia.
  void shutdown_callback();
};

static std::unique_ptr<GaiaBot> m_ptr{nullptr};

}   // namespace gaia_bot

#endif  // GAIA_BOT__GAIA_BOT_HPP_
