#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <string>
#include <vector>

#include "gaia_access_control.h"
#include "communication.hpp"
#include "enums.hpp"
#include "helpers.hpp"
#include "json.hpp"

#include "gaia/db/db.hpp"
#include "gaia/logger.hpp"
#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

using json = nlohmann::json;
using namespace gaia::access_control;

void exit_callback(int signal_number)
{
    gaia::system::shutdown();
    std::cout << std::endl
              << "Exiting." << std::endl;
    exit(signal_number);
}

json get_event_json(event_t event)
{
    json j;

    j["name"] = event.name();
    j["start_timestamp"] = event.start_timestamp();
    j["end_timestamp"] = event.end_timestamp();
    j["room_name"] = event.held_in_room().name();

    return j;
}

json get_person_json(person_t person)
{
    json j;

    j["person_id"] = person.person_id();
    j["first_name"] = person.first_name();
    j["employee"] = person.employee();
    j["visitor"] = person.visitor();
    j["stranger"] = person.stranger();
    j["badged"] = person.badged();
    j["parked"] = person.parked();
    j["credentialed"] = person.credentialed();
    j["admissible"] = person.admissible();
    j["on_wifi"] = person.on_wifi();

    j["events"] = json::array();
    for (auto reg_iter = person.registrations().begin();
         reg_iter != person.registrations().end();
         reg_iter++)
    {
        j["events"].push_back(get_event_json(reg_iter->occasion()));
    }

    if (person.inside_room())
    {
        j["inside_room"] = person.inside_room().name();
    }

    return j;
}

json get_room_json(room_t room)
{
    json j;

    j["room_id"] = room.room_id();
    j["name"] = room.name();
    j["capacity"] = room.capacity();
    j["people"] = json::array();

    for (auto person_iter = room.people_inside().begin();
         person_iter != room.people_inside().end();
         person_iter++)
    {
        j["people"].push_back(get_person_json(*person_iter));
    }

    j["events"] = json::array();
    for (auto event_iter = room.events().begin();
         event_iter != room.events().end();
         event_iter++)
    {
        j["events"].push_back(get_event_json(*event_iter));
    }

    return j;
}

json get_building_json(building_t building)
{
    json j;

    j["building_id"] = building.building_id();
    j["name"] = building.name();
    j["rooms"] = json::array();
    for (auto room_iter = building.rooms().begin();
         room_iter != building.rooms().end();
         room_iter++)
    {
        j["rooms"].push_back(get_room_json(*room_iter));
    }

    j["people"] = json::array();

    for (auto person_iter = building.people_entered().begin();
         person_iter != building.people_entered().end();
         person_iter++)
    {
        if (!person_iter->inside_room()) {
            j["people"].push_back(get_person_json(*person_iter));
        }
    }

    return j;
}

json get_init_json()
{
    json j;
    
    j["buildings"] = json::array();
    for (const auto& building : building_t::list())
    {
        j["buildings"].push_back(get_building_json(building));
    }

    j["people"] = json::array();
    for (const auto& person : person_t::list())
    {
        j["people"].push_back(get_person_json(person));
    }

    return j;
}

void update_ui()
{
    gaia::db::begin_transaction();

    json j;

    j["buildings"] = json::array();

    for (auto building_iter = building_t::list().begin();
         building_iter != building_t::list().end();
         building_iter++)
    {
        j["buildings"].push_back(get_building_json(*building_iter));
    }

    j["people"] = json::array();
    for (auto person_iter = person_t::list().begin();
         person_iter != person_t::list().end();
         person_iter++)
    {
        if (!person_iter->inside_room() && !person_iter->entered_building())
        {
            j["people"].push_back(get_person_json(*person_iter));
        }
    }

    communication::publish_message("access_control_json", j.dump());

    gaia::db::commit_transaction();
}

event_t add_event(std::string name, uint64_t start_timestamp,
                    uint64_t end_timestamp, room_t room)
{
    auto event_w = event_writer();
    event_w.name = name;
    event_w.start_timestamp = start_timestamp;
    event_w.end_timestamp = end_timestamp;
    event_t new_event = event_t::get(event_w.insert_row());

    room.events().insert(new_event);

    return new_event;
}

registration_t add_registration(person_t person, event_t occasion)
{
    registration_t registration = registration_t::get(registration_t::insert_row("", 0));

    occasion.registrations().insert(registration);
    person.registrations().insert(registration);

    return registration;
}

room_t add_room(uint64_t room_id, std::string room_name,
              uint32_t capacity, building_t building)
{
    auto room_w = room_writer();
    room_w.room_id = room_id;
    room_w.name = room_name;
    room_w.capacity = capacity;
    room_t new_room = room_t::get(room_w.insert_row());

    building.rooms().insert(new_room);

    return new_room;
}

person_t add_person(uint64_t person_id, std::string first_name,
                bool employee, bool visitor, bool stranger)
{
    auto person_w = person_writer();
    person_w.person_id = person_id;
    person_w.first_name = first_name;
    person_w.employee = employee;
    person_w.visitor = visitor;
    person_w.stranger = stranger;
    person_w.entry_time = 0;
    person_w.leave_time = 100000;
    return person_t::get(person_w.insert_row());
}

void populate_all_tables()
{
    using namespace gaia::access_control;

    helpers::set_time(480);

    person_t john = add_person(1, "John", true, false, false);
    person_t jane = add_person(2, "Jane", false, true, false);
    person_t stranger = add_person(3, "Mr. Stranger", false, false, true);

    // Headquarters building.
    auto headquarters_w = building_writer();
    headquarters_w.building_id = 10;
    headquarters_w.name = "HQ Building";
    building_t headquarters = building_t::get(headquarters_w.insert_row());

    room_t room;
    event_t event;

    room = add_room(102, "Auditorium", 200, headquarters);
    event = add_event("Happy hour", 720, 840, room);
    add_registration(john, event);
    add_registration(jane, event);

    room = add_room(104, "Big Room", 4, headquarters);
    event = add_event("The best meeting", 660, 720, room);
    event = add_event("Boring meeting", 600, 660, room);
    add_registration(john, event);
    add_registration(jane, event);

    room = add_room(103, "Little Room", 3, headquarters);
    event = add_event("Important meeting", 540, 600, room);
    add_registration(john, event);
}

void clear_all_tables()
{
    using namespace gaia::access_control;

    for (auto building = *building_t::list().begin(); building; building = *building_t::list().begin())
    {
        building.rooms().clear();
        building.parked_people().clear();
        building.people_entered().clear();
        building.scans().clear();
        building.delete_row();
    }
    for (auto room = *room_t::list().begin(); room; room = *room_t::list().begin())
    {
        room.people_inside().clear();
        room.permissions().clear();
        room.events().clear();
        room.scans().clear();
        room.delete_row();
    }
    for (auto person = *person_t::list().begin(); person; person = *person_t::list().begin())
    {
        person.permitted_in().clear();
        person.registrations().clear();
        person.vehicles().clear();
        person.scans().clear();
        person.delete_row();
    }
    for (auto permitted_room = *permitted_room_t::list().begin(); permitted_room;
         permitted_room = *permitted_room_t::list().begin())
    {
        permitted_room.delete_row();
    }
    for (auto event = *event_t::list().begin(); event; event = *event_t::list().begin())
    {
        event.registrations().clear();
        event.delete_row();
    }
    for (auto registration = *registration_t::list().begin(); registration;
         registration = *registration_t::list().begin())
    {
        registration.delete_row();
    }
    for (auto vehicle = *vehicle_t::list().begin(); vehicle; vehicle = *vehicle_t::list().begin())
    {
        vehicle.scans().clear();
        vehicle.delete_row();
    }
    for (auto scan = *scan_t::list().begin(); scan; scan = *scan_t::list().begin())
    {
        scan.delete_row();
    }
}

bool get_person(uint64_t person_id, person_t& person)
{
    auto person_iter = person_t::list().where(person_t::expr::person_id == person_id).begin();
    if (person_iter == person_t::list().end())
    {
        return false;
    }

    person = *person_iter;  
    return true;
}

bool get_room(uint64_t room_id, room_t& room)
{
    auto room_iter = room_t::list().where(room_t::expr::room_id == room_id).begin();
    if (room_iter == room_t::list().end())
    {
        return false;
    }

    room = *room_iter;  
    return true;
}

bool get_building(uint64_t building_id, building_t& building)
{
    auto building_iter = building_t::list().where(building_t::expr::building_id == building_id).begin();
    if (building_iter == building_t::list().end())
    {
        return false;
    }

    building = *building_iter;  
    return true;
}

void add_scan(const json &j)
{
    using namespace enums::scan_table;
    gaia::db::begin_transaction();

    auto scan_w = scan_writer();
    if (j["scan_type"] == "badge")
    {
        scan_w.scan_type = e_scan_type::badge;
    } 
    else if (j["scan_type"] == "vehicle_entering")
    {
        scan_w.scan_type = e_scan_type::vehicle_entering;
    } 
    else if (j["scan_type"] == "vehicle_departing")
    {
        scan_w.scan_type = e_scan_type::vehicle_departing;
    } 
    else if (j["scan_type"] == "joining_wifi")
    {
        scan_w.scan_type = e_scan_type::joining_wifi;
    } 
    else if (j["scan_type"] == "leaving_wifi")
    {
        scan_w.scan_type = e_scan_type::leaving_wifi;
    } 
    else if (j["scan_type"] == "face")
    {
        scan_w.scan_type = e_scan_type::face;
    } 
    else if (j["scan_type"] == "leaving")
    {
        scan_w.scan_type = e_scan_type::leaving;
    } 

    scan_t new_scan = scan_t::get(scan_w.insert_row());

    person_t person;
    if (get_person(j["person_id"], person))
    {
        person.scans().insert(new_scan);
    }

    room_t room;
    if (!j["room_id"].is_null() && get_room(j["room_id"], room))
    {
        room.scans().insert(new_scan);
        room.building().scans().insert(new_scan);
    }

    building_t building;
    if (!j["building_id"].is_null() && get_building(j["building_id"], building))
    {
        building.scans().insert(new_scan);
    }

    gaia::db::commit_transaction();
}

std::vector<std::string> split_topic(const std::string& topic)
{
    std::vector<std::string> result;
    size_t left = 0;
    size_t right = topic.find('/');
    while (right != std::string::npos)
    {
        result.push_back(topic.substr(left, right - left));
        left = right + 1;
        right = topic.find('/', left);
    }
    result.push_back(topic.substr(left));
    return result;
}

void message_callback(const std::string &topic, const std::string &payload)
{
    std::vector<std::string> topic_vector = split_topic(topic);
    if (gaia_log::app().is_debug_enabled())
    {
        gaia_log::app().debug("Received topic: {} | payload: {}", topic, payload);
    }
    
    if (topic_vector.size() < 2 || topic_vector.at(1) != "access_control")
    {
        gaia_log::app().error("Unexpected topic: {}", topic);
        return;
    }

    if (topic_vector.at(2) == "time")
    {
        int64_t time = std::stoll(payload);
        if (time < 0)
        {
            gaia_log::app().error("Tried to set a negative time: {}", time);
        }
        else
        {
            helpers::set_time(time);
        }
    }
    else if (topic_vector.at(2) == "scan")
    {
        add_scan(json::parse(payload));
    }
    else
    {
        gaia_log::app().error("Unexpected topic: {}", topic);
        return;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, exit_callback);
    gaia::system::initialize();

    if (!communication::init(argc, argv))
    {
        exit_callback(EXIT_FAILURE);
    }

    gaia::db::begin_transaction();
    clear_all_tables();
    populate_all_tables();
    std::string init_msg = get_init_json().dump();
    gaia::db::commit_transaction();

    communication::connect(message_callback, init_msg);
    exit_callback(EXIT_SUCCESS);
}