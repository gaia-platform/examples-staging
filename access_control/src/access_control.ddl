----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

create database if not exists access_control;

use access_control;

create table if not exists building (
    building_id uint64,
    name string
);

create table if not exists room (
    room_id uint64,
    name string,
    capacity uint32
);

create relationship if not exists rooms_inside_building (
    building.rooms -> room[],
    room.building -> building
);

create table if not exists person (
    person_id uint64,
    face_signature string,
    badge_number string,
    first_name string,
    last_name string,

    employee bool,
    visitor bool,
    stranger bool,
    badged bool,
    parked bool,
    credentialed bool,
    admissible bool,
    on_wifi bool,

    entry_time uint64,
    leave_time uint64
);

create relationship if not exists person_parked_in_building (
    building.parked_people -> person[],
    person.parked_in -> building
);

create relationship if not exists person_entered_building (
    building.people_entered -> person[],
    person.entered_building -> building
);

create relationship if not exists person_inside_room (
    room.people_inside -> person[],
    person.inside_room -> room
);

create table if not exists permitted_room (
    permitted_room_id uint64
);

create relationship if not exists permitted_room_permittee (
    person.permitted_in -> permitted_room[],
    permitted_room.permittee -> person
);

create relationship if not exists permitted_room_allowed_in (
    room.permissions -> permitted_room[],
    permitted_room.allowed_in -> room
);

create table if not exists event (
    event_id string,
    name string,
    start_timestamp uint64,
    end_timestamp uint64
);

create relationship if not exists event_held_in_room (
    room.events -> event[],
    event.held_in_room -> room
);

create table if not exists registration (
    registration_id string,
    timestamp uint64
);

create relationship if not exists person_registration (
    person.registrations -> registration[],
    registration.registered -> person
);

create relationship if not exists event_registration (
    event.registrations -> registration[],
    registration.occasion -> event
);

create table if not exists vehicle (
    vehicle_id uint64,
    make string,
    model string,
    year string,
    license_state string,
    license string,
    parked_time uint64
);

create relationship if not exists vehicle_owner (
    person.vehicles -> vehicle[],
    vehicle.owner -> person
);

create table if not exists scan (
    scan_id uint64,
    scan_type uint8,
    badge_scan bool,
    vehicle_entering bool,
    vehicle_departing bool,
    joining_wifi bool,
    leaving_wifi bool,
    face_scan bool,
    leaving bool,
    badge_id string,
    face_signature string,
    license string,
    timestamp uint64
);

create relationship if not exists scan_seen_at (
    building.scans -> scan[],
    scan.seen_at_building -> building
);

create relationship if not exists scan_seen_in (
    room.scans -> scan[],
    scan.seen_in_room -> room
);

create relationship if not exists scan_seen_who (
    person.scans -> scan[],
    scan.seen_who_person -> person
);

create relationship if not exists scan_seen_license (
    vehicle.scans -> scan[],
    scan.seen_license_vehicle -> vehicle
);
