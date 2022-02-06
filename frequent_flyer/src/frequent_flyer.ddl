---------------------------------------------
-- Copyright (c) 2022 Gaia Platform LLC
--
-- Use of this source code is governed by an MIT-style
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
---------------------------------------------

database frequent_flyer

table airports (
    -- Fields
    city string,

    -- References
    departure_routes references routes[],
    arrival_routes references routes[],
    departure_trips references trips[],
    arrival_trips references trips[]
)

table airplanes (
    -- Fields
    tail_number uint32 unique,
    max_weight uint32,

    -- References
    flights references flights[]
)

table routes (
    -- Fields
    route_miles uint32,

    -- References
    departure_airport references airports
        using departure_routes,
    arrival_airport references airports
        using arrival_routes,
    flights references flights[]
)

table flights (
    -- Fields
    id uint32 unique,
    flight_number uint32,
    flight_date uint64,
    flight_miles uint32,
    flight_status string,
    segments_flown uint8,

    -- References
    airplane references airplanes,
    route references routes,
    segments references segments[]
)

table segments (
    -- Fields
    segment_order uint8,
    flight_id uint32,
    trip_id uint32,

    -- References
    flight references flights
        where segments.flight_id = flights.id,
    trip references trips
        where segments.trip_id = trips.id
)

table trips (
    -- Fields
    id uint32 unique,
    traveler_id uint32,
    description string,
    trip_miles uint32,
    segments_flown uint8,

    -- References
    traveler references travelers,
    segments references segments[],
    departure_airport references airports
        using departure_trips,
    arrival_airport references airports
        using arrival_trips
)

table travelers (
    -- Fields
    id uint32 unique,
    first_name string,
    surname string,
    lifetime_miles uint32,
    member_level string,

    -- References
    trips references trips[]
        where travelers.id = trips.traveler_id
)
