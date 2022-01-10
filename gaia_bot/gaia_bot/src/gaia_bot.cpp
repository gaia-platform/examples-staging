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

#include "gaia_bot/gaia_bot.hpp"
#include "gaia/logger.hpp"

#include <string>

using gaia::gaia_bot::ego_t;
using gaia::gaia_bot::range_sensors_t;
using sensor_msgs::msg::Range;
using vision_msgs::msg::Detection3D;
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

namespace gaia_bot
{

GaiaBot::GaiaBot(const rclcpp::NodeOptions & options)
:   Node("gaia_bot", options)
{
  // Make this Node instance available for the ruleset.
  m_ptr.reset(this);

  // Initialize Gaia.
  gaia::system::initialize();

  rclcpp::on_shutdown(
    [this]
    {
      GaiaBot::shutdown_callback();
    });

  // Initialize database.
  gaia::db::begin_transaction();
  if (ego_t::list().size() == 0)
  {
    ego_t::insert_row(0);
  }
  gaia::db::commit_transaction();

  // Create publishers and subscribers.
  using std::placeholders::_1;

  m_joint_trajectory_pub = create_publisher<JointTrajectory>("neck_pose", rclcpp::QoS(10));

  m_range_sub = create_subscription<Range>(
    "range", rclcpp::QoS(10),
    std::bind(&GaiaBot::range_callback, this, _1));

  m_face_detection_sub = create_subscription<Detection3D>(
    "face", rclcpp::QoS(10),
    std::bind(&GaiaBot::face_detection_callback, this, _1));
}

GaiaBot::~GaiaBot()
{
  m_ptr.release();
}

void GaiaBot::publish_neck_pose(float rotation, float lift)
{
  JointTrajectory msg;

  msg.joint_names.push_back("rotation");
  msg.joint_names.push_back("lift");

  JointTrajectoryPoint jtpRotation;
  jtpRotation.positions.push_back(rotation);

  JointTrajectoryPoint jtpLift;
  jtpLift.positions.push_back(lift);
  
  msg.points.push_back(jtpRotation);
  msg.points.push_back(jtpLift);

  m_ptr->m_joint_trajectory_pub->publish(msg);
}

void GaiaBot::range_callback(const Range::ConstSharedPtr msg)
{
  gaia::db::begin_transaction();
  // TODO: Generalize to handle multiple range sensors
  auto range_sensor_iter = range_sensors_t::list().where(range_sensors_t::expr::id == 0).begin();
  if (range_sensor_iter == range_sensors_t::list().end())
  {
    range_sensors_t::insert_row(0, "main_range_sensor", msg->range, 0);
  }
  else
  {
    auto range_sensor_writer = range_sensor_iter->writer();
    range_sensor_writer.distance = msg->range;
    range_sensor_writer.update_row();
  }
  gaia::db::commit_transaction();
}

void GaiaBot::face_detection_callback(const Detection3D::ConstSharedPtr msg)
{
  gaia_log::app().info("Detection Message {}, {}", msg->bbox.center.position.x, msg->bbox.center.position.y);
  // TODO: Handle face detections
}

void GaiaBot::shutdown_callback()
{
  gaia::system::shutdown();
}

}  // namespace gaia_bot

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gaia_bot::GaiaBot)
