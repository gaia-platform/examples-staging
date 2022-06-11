----------------------------------------------------
-- Copyright (c) Gaia Platform Authors
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database frequent_flyer

table airport (
    -- Fields
    airport_code string unique,

    -- References
    departure_routes references route[],
    arrival_routes references route[],
    departure_trips references trip[],
    arrival_trips references trip[]
)

table airplane (
    -- Fields
    tail_number uint32 unique,
    max_weight uint32,

    -- References
    flights references flight[]
)

table route (
    -- Fields
    route_miles uint32,
    departure_airport_code string,
    arrival_airport_code string,

    -- References
    departure_airport references airport
        using departure_routes
            where route.departure_airport_code = airport.airport_code,
    arrival_airport references airport
        using arrival_routes
            where route.arrival_airport_code = airport.airport_code,
    flights references flight[]
)

table flight (
    -- Fields
    flight_number uint32 unique,
    flight_date uint64,
    flight_miles uint32,
    flight_status string,
    airplane_tail_number uint32,
    segments_flown uint8,

    -- References
    airplane references airplane
        where flight.airplane_tail_number = airplane.tail_number,
    route references route,
    segments references segment[]
)

table segment (
    -- Fields
    segment_order uint8,
    flight_number uint32,
    trip_id uint32,

    -- References
    flight references flight
        where segment.flight_number = flight.flight_number,
    trip references trip
        where segment.trip_id = trip.id
)

table trip (
    -- Fields
    id uint32 unique,
    traveler_id uint32,
    description string,
    trip_miles uint32,
    segments_flown uint8,
    departure_airport_code string,
    arrival_airport_code string,

    -- References
    traveler references traveler,
    segments references segment[],
    departure_airport references airport
        using departure_trips
            where trip.departure_airport_code = airport.airport_code,
    arrival_airport references airport
        using arrival_trips
            where trip.arrival_airport_code = airport.airport_code
)

table traveler (
    -- Fields
    id uint32 unique,
    first_name string,
    surname string,
    lifetime_miles uint32,
    member_level string,

    -- References
    trips references trip[]
        where traveler.id = trip.traveler_id
)
