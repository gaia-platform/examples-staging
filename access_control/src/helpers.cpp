////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <chrono>
#include <limits>
#include <random>
#include <vector>

#include "communication.hpp"
#include "helpers.hpp"

using namespace gaia::access_control;

std::string helpers::scan_type_string(enums::scan_table::e_scan_type scan_type)
{
    using namespace enums::scan_table;
    switch (scan_type)
    {
        case badge : return "badge";
        case vehicle_entering : return "vehicle_entering";
        case vehicle_departing : return "vehicle_departing";
        case joining_wifi : return "joining_wifi";
        case leaving_wifi : return "leaving_wifi";
        case face : return "face";
        case leaving : return "leaving";
        default : return "";
    }
}

void helpers::park_in_building(
    gaia::common::gaia_id_t person_id,
    gaia::common::gaia_id_t building_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    auto building = gaia::access_control::building_t::get(building_id);

    person.registrations();

    building.parked_people().insert(person);
}

bool helpers::person_has_registrations(gaia::common::gaia_id_t person_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    auto registration_list = person.registrations();

    return (registration_list.begin() != registration_list.end());
}

bool helpers::person_has_event_now(gaia::common::gaia_id_t person_id)
{
    auto person = gaia::access_control::person_t::get(person_id);

    for(auto registration : person.registrations())
    {
        auto event = registration.occasion();
        if (event && time_is_between(get_time_now(),
            event.start_timestamp(), event.end_timestamp()))
        {
            return true;
        }
    }
    return false;
}

bool helpers::person_has_event_now(
    gaia::common::gaia_id_t person_id,
    gaia::access_control::room_t room)
{
    auto person = gaia::access_control::person_t::get(person_id);

    for(auto registration : person.registrations())
    {
        auto event = registration.occasion();
        if((!room || room == event.held_in_room())
            && event && time_is_between(get_time_now(),
            event.start_timestamp(), event.end_timestamp()))
        {
            return true;
        }
    }
    return false;
}

static std::random_device rand_device;
static std::mt19937 rand_num_gen{rand_device()};
static std::uniform_int_distribution<uint64_t> distribution(
    std::numeric_limits<uint64_t>::min(), std::numeric_limits<uint64_t>::max());

void helpers::allow_person_into_room(
    gaia::common::gaia_id_t person_id,
    gaia::access_control::room_t room)
{
    uint64_t permitted_room_id = distribution(rand_num_gen);
    auto permitted_room = gaia::access_control::permitted_room_t::insert_row(permitted_room_id);

    auto person = gaia::access_control::person_t::get(person_id);
    // Connect permitted_room to person.
    person.permitted_in().insert(permitted_room);
    // Connect permitted_room to room.
    room.permissions().insert(permitted_room);
}

void helpers::disconnect_parked_buildings(gaia::access_control::vehicle_t vehicle)
{
    auto vehicle_owner = vehicle.owner();
    auto parking_building = vehicle_owner.parked_in();

    parking_building.parked_people().remove(vehicle_owner);
}

void helpers::disconnect_person_from_room(gaia::common::gaia_id_t person_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    if (person.inside_room()) {
        std::string building_id = std::to_string(person.inside_room().building().building_id());
        std::string topic = "access_control/" + std::to_string(person.person_id()) + "/move_to_building";

        person.inside_room().people_inside().remove(person);

        // Move the person back into the building but not a specific room.
        communication::publish_message(topic, building_id);
    }
}

void helpers::disconnect_person_from_building(gaia::common::gaia_id_t person_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    if (person.entered_building()) {
        person.entered_building().people_entered().remove(person);

        std::string topic = "access_control/" + std::to_string(person.person_id()) + "/move_to_building";
        communication::publish_message(topic, "");
    }
}

void helpers::delete_their_room_permissions(gaia::common::gaia_id_t person_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    std::vector<gaia::access_control::permitted_room_t> permitted_rooms;
    for(auto permitted_room : person.permitted_in())
    {
        permitted_rooms.push_back(permitted_room);
    }

    person.permitted_in().clear();
    for(auto permitted_room : permitted_rooms)
    {
        permitted_room.delete_row();
    }
}

void helpers::let_them_in(
    gaia::common::gaia_id_t person_id,
    gaia::common::gaia_id_t scan_id)
{
    auto person = gaia::access_control::person_t::get(person_id);
    auto scan = gaia::access_control::scan_t::get(scan_id);

    if (scan.seen_in_room())
    {
        disconnect_person_from_room(person_id);
        scan.seen_in_room().people_inside().insert(person);

        std::string building_and_room = std::to_string(scan.seen_at_building().building_id());
        building_and_room.append(",");
        building_and_room.append(std::to_string(scan.seen_in_room().room_id()));

        std::string topic = "access_control/" + std::to_string(person.person_id()) + "/move_to_room";
        communication::publish_message(topic, building_and_room);
    }
    if (scan.seen_at_building())
    {
        scan.seen_at_building().people_entered().insert(person);

        if(!scan.seen_in_room())
        {
            std::string building_id = std::to_string(scan.seen_at_building().building_id());
            std::string topic = "access_control/" + std::to_string(person.person_id()) + "/move_to_building";
            communication::publish_message(topic, building_id);
        }
    }
}

gaia::access_control::person_t helpers::insert_stranger(std::string face_signature)
{
    auto stranger_w = gaia::access_control::person_writer();
    stranger_w.stranger = true;
    stranger_w.face_signature = face_signature;

    return gaia::access_control::person_t::get(stranger_w.insert_row());
}

void helpers::insert_stranger_vehicle(
    gaia::access_control::person_t stranger, std::string license)
{
    auto vehicle_w = gaia::access_control::vehicle_writer();
    vehicle_w.license = license;
    // Connect the vehicle to its owner, the stranger.
    stranger.vehicles().insert(vehicle_w.insert_row());
}

void helpers::send_updated_scan(gaia::access_control::person_t person,
    uint8_t scan_type)
{
    auto scan_type_enum = static_cast<enums::scan_table::e_scan_type>(scan_type);
    if (scan_type_enum != enums::scan_table::face && scan_type_enum != enums::scan_table::leaving)
    {
        std::string topic = "access_control/" + std::to_string(person.person_id()) + "/scan";
        std::string scan_type_str = helpers::scan_type_string(scan_type_enum);
        communication::publish_message(topic, scan_type_str);
    }
}

// Time-related helpers:

uint64_t g_current_time = 0;

void helpers::set_time(uint64_t time)
{
    g_current_time = time;
}

uint64_t helpers::get_time_now()
{
    return g_current_time;
}

bool helpers::time_is_between(uint64_t time,
    uint64_t low_time, uint64_t high_time)
{
    return (low_time <= time) && (time <= high_time);
}
