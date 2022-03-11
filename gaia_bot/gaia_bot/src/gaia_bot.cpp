// Copyright 2022 Gaia Platform, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gaia_bot/gaia_bot.hpp"
#include "gaia/logger.hpp"

#include <string>
#include <thread>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;
using std::thread;
using std::this_thread::sleep_for;
using gaia::db::begin_transaction;
using gaia::db::commit_transaction;
using gaia::gaia_bot::ego_t;
using gaia::gaia_bot::neck_pose_t;
using gaia::gaia_bot::neck_pose_writer;
using gaia::gaia_bot::range_sensors_t;
using gaia::gaia_bot::face_detections_t;
using gaia::gaia_bot::face_tracks_t;
using sensor_msgs::msg::BatteryState;
using sensor_msgs::msg::Range;
using sensor_msgs::msg::Illuminance;
using vision_msgs::msg::Detection3D;
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

namespace gaia_bot
{

bool GaiaBot::s_running = true;

uint64_t get_time_millis()
{
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

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

    // Create publishers and subscribers.
    using std::placeholders::_1;

    m_joint_trajectory_pub = create_publisher<JointTrajectory>("neck_pose", rclcpp::QoS(10));

    m_left_light_sub = create_subscription<Illuminance>(
        "left_light", rclcpp::QoS(10),
        std::bind(&GaiaBot::left_light_callback, this, _1));

    m_right_light_sub = create_subscription<Illuminance>(
        "right_light", rclcpp::QoS(10),
        std::bind(&GaiaBot::right_light_callback, this, _1));

    m_battery_state_sub = create_subscription<BatteryState>(
        "battery", rclcpp::QoS(10),
        std::bind(&GaiaBot::battery_state_callback, this, _1));

    m_range_sub = create_subscription<Range>(
        "range", rclcpp::QoS(10),
        std::bind(&GaiaBot::range_callback, this, _1));

    m_face_detection_sub = create_subscription<Detection3D>(
        "face", rclcpp::QoS(10),
        std::bind(&GaiaBot::face_detection_callback, this, _1));

    // Initialize database.
    begin_transaction();
    if (ego_t::list().size() == 0)
    {
        auto ego = ego_t::get(ego_t::insert_row(1, "gaia_bot", get_time_millis()));
        neck_pose_t::insert_row(0.0, 0.0);
    }
    else
    {
        auto neck_pose = *neck_pose_t::list().begin();
        if (neck_pose)
        {
            neck_pose_writer w = neck_pose.writer();
            w.rotation = 0.0;
            w.lift = 0.0;
            w.update_row();
        }
    }
    commit_transaction();

    thread ego_time_update_thread(ego_time_update);
    ego_time_update_thread.detach();
}

GaiaBot::~GaiaBot()
{
    m_ptr.release();
}

void GaiaBot::ego_time_update()
{
    gaia::system::initialize();

    while (s_running)
    {
        begin_transaction();
        auto ego_iter = ego_t::list().where(ego_t::expr::id == 1).begin();
        if (ego_iter != ego_t::list().end())
        {
            auto ego_writer = ego_iter->writer();
            ego_writer.now = get_time_millis();
            ego_writer.update_row();
        }
        commit_transaction();

        sleep_for(c_ego_time_update_interval);
    }
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

void GaiaBot::left_light_callback(const Illuminance::ConstSharedPtr msg)
{
    gaia_log::app().info("Left light Illuminance {}", msg->illuminance);
    // TODO: Handle left light illuminance
}

void GaiaBot::right_light_callback(const Illuminance::ConstSharedPtr msg)
{
    gaia_log::app().info("Right light Illuminance {}", msg->illuminance);
    // TODO: Handle right light illuminance
}

void GaiaBot::battery_state_callback(const BatteryState::ConstSharedPtr msg)
{
    gaia_log::app().info("BatteryState voltage {}", msg->voltage);
    // TODO: Handle battery state
}

void GaiaBot::range_callback(const Range::ConstSharedPtr msg)
{
    begin_transaction();
    // TODO: Generalize to handle multiple range sensors
    auto range_sensor_iter = range_sensors_t::list().where(range_sensors_t::expr::id == 0).begin();
    if (range_sensor_iter == range_sensors_t::list().end())
    {
        range_sensors_t::insert_row(0, "main_range_sensor", get_time_millis(), msg->range, 1);
    }
    else
    {
        auto range_sensor_writer = range_sensor_iter->writer();
        range_sensor_writer.distance = msg->range;
        range_sensor_writer.timestamp = get_time_millis();
        range_sensor_writer.update_row();
    }
    commit_transaction();
}

void GaiaBot::face_detection_callback(const Detection3D::ConstSharedPtr msg)
{
    try
    {
        begin_transaction();
        face_detections_t::insert_row(
            get_time_millis(), msg->bbox.center.position.x, msg->bbox.center.position.y, msg->bbox.center.position.z,
            msg->bbox.size.x, msg->bbox.size.y, msg->bbox.size.z, false, 0);
        commit_transaction();
    }
    catch (const gaia::db::transaction_update_conflict& ex)
    {
        gaia_log::app().error("transaction_update_conflict");
    }
}

void GaiaBot::shutdown_callback()
{
    s_running = false;
    gaia::system::shutdown();
}

}  // namespace gaia_bot

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gaia_bot::GaiaBot)
