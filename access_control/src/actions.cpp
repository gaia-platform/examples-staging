////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include "actions.hpp"

#include "communication.hpp"

#include "gaia/logger.hpp"

using namespace gaia::access_control;

const std::string c_alert_topic = "access_control/alert";

void actions::stranger_detected()
{
    gaia_log::app().info("Stranger detected!");
    communication::publish_message(c_alert_topic, "Stranger detected");
}

void actions::no_eligible_events(uint64_t person_id)
{
    gaia_log::app().info("No eligible events for visitor #{}.", person_id);
    communication::publish_message(c_alert_topic, "No eligible events for visitor");
}

void actions::base_credentials_required(uint64_t person_id)
{
    gaia_log::app().info("Base credentials required for person #{} to enter.", person_id);
    communication::publish_message(c_alert_topic, "Base credentials required to enter");
}

void actions::no_entry_right_now(
    uint64_t person_id, std::string room_name, std::string building_name)
{
    gaia_log::app().info("No entry right now for person #{} into room {}, building {}.",
        person_id, room_name, building_name);
    communication::publish_message(c_alert_topic, "Entry not currently allowed");
}

void actions::not_this_building(uint64_t person_id, std::string building_name)
{
    gaia_log::app().info("Person #{} is not allowed into building {}.",
        person_id, building_name);
    communication::publish_message(c_alert_topic, "Entry into building not allowed");
}

void actions::not_this_room(
    uint64_t person_id, std::string room_name, std::string building_name)
{
    gaia_log::app().info("Person #{} is not allowed into room {}, building {}.",
        person_id, room_name, building_name);
    communication::publish_message(c_alert_topic, "Entry into room not allowed");
}
