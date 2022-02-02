/////////////////////////////////////////////
// Copyright (c) 2022 Gaia Platform LLC
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
/////////////////////////////////////////////

#include <chrono>
#include <iostream>

#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "gaia_frequent_flyer.h"

using namespace gaia::common;
using namespace gaia::db;
using namespace gaia::db::triggers;
using namespace gaia::direct_access;
using namespace gaia::frequent_flyer;
using namespace gaia::rules;

using std::chrono::milliseconds;
using std::chrono::system_clock;
using std::chrono::duration_cast;

template <class T_table>
void delete_all_rows()
{
    for (auto obj = *T_table::list().begin();
         obj; obj = *T_table::list().begin())
    {
        obj.delete_row();
    }
}

uint64_t get_time_millis()
{
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void remove_all_data()
{
    for (auto airplane = *airplanes_t::list().begin();
        airplane; airplane = *airplanes_t::list().begin())
    {
        airplane.flights().clear();
        airplane.delete_row();
    }

    for (auto airport = *airports_t::list().begin();
        airport; airport = *airports_t::list().begin())
    {
        airport.departure_routes().clear();
        airport.arrival_routes().clear();
        airport.departure_trips().clear();
        airport.arrival_trips().clear();
        airport.delete_row();
    }

    for (auto route = *routes_t::list().begin();
        route; route = *routes_t::list().begin())
    {
        route.flights().clear();
        route.delete_row();
    }

    for (auto segment = *segments_t::list().begin();
        segment; segment = *segments_t::list().begin())
    {
        auto segment_writer = segment.writer();
        segment_writer.flight_id = 0;
        segment_writer.trip_id = 0;
        segment_writer.update_row();
        segment.delete_row();
    }

    for (auto trip = *trips_t::list().begin();
        trip; trip = *trips_t::list().begin())
    {
        auto trip_writer = trip.writer();
        trip_writer.traveler_id = 0;
        trip_writer.update_row();
        trip.delete_row();
    }

    delete_all_rows<flights_t>();
    delete_all_rows<travelers_t>();
}

void init_storage()
{
    auto_transaction_t tx(auto_transaction_t::no_auto_begin);

    remove_all_data();

    auto route_seattle_la = routes_t::get(routes_t::insert_row());
    auto route_la_seattle = routes_t::get(routes_t::insert_row());
    auto route_la_ohare = routes_t::get(routes_t::insert_row());
    auto route_ohare_la = routes_t::get(routes_t::insert_row());
    auto route_ohare_seattle = routes_t::get(routes_t::insert_row());
    auto route_ohare_boston = routes_t::get(routes_t::insert_row());
    auto route_boston_seattle = routes_t::get(routes_t::insert_row());

    auto airplane_1 = airplanes_t::get(airplanes_t::insert_row(1, 100000));
    auto airplane_2 = airplanes_t::get(airplanes_t::insert_row(2, 90000));
    auto airplane_3 = airplanes_t::get(airplanes_t::insert_row(3, 50000));
    auto airplane_4 = airplanes_t::get(airplanes_t::insert_row(4, 120000));

    auto airport_seattle = airports_t::get(airports_t::insert_row("Seattle"));
    airport_seattle.departure_routes().insert(route_seattle_la);
    airport_seattle.arrival_routes().insert(route_la_seattle);
    airport_seattle.arrival_routes().insert(route_ohare_seattle);
    airport_seattle.arrival_routes().insert(route_boston_seattle);

    auto airport_la = airports_t::get(airports_t::insert_row("Los Angeles"));
    airport_la.departure_routes().insert(route_la_seattle);
    airport_la.departure_routes().insert(route_la_ohare);
    airport_la.arrival_routes().insert(route_seattle_la);
    airport_la.arrival_routes().insert(route_ohare_la);

    auto airport_ohare = airports_t::get(airports_t::insert_row("O'Hare"));
    airport_ohare.departure_routes().insert(route_ohare_la);
    airport_ohare.departure_routes().insert(route_ohare_seattle);
    airport_ohare.departure_routes().insert(route_ohare_boston);
    airport_ohare.arrival_routes().insert(route_la_ohare);

    auto airport_boston = airports_t::get(airports_t::insert_row("Boston"));
    airport_ohare.departure_routes().insert(route_boston_seattle);
    airport_ohare.arrival_routes().insert(route_ohare_boston);

    auto flight = flights_t::get(flights_t::insert_row(1, 111, 0, 0, "scheduled", 0));
    airplane_1.flights().insert(flight);
    route_seattle_la.flights().insert(flight);
    route_la_ohare.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(2, 222, 0, 0, "scheduled", 0));
    airplane_2.flights().insert(flight);
    route_la_ohare.flights().insert(flight);
    route_ohare_boston.flights().insert(flight);
    route_boston_seattle.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(3, 333, 0, 0, "scheduled", 0));
    airplane_3.flights().insert(flight);
    route_ohare_seattle.flights().insert(flight);
    route_seattle_la.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(4, 444, 0, 0, "scheduled", 0));
    airplane_4.flights().insert(flight);
    route_seattle_la.flights().insert(flight);
    route_la_seattle.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(5, 555, 0, 0, "scheduled", 0));
    airplane_4.flights().insert(flight);
    route_ohare_la.flights().insert(flight);
    route_la_ohare.flights().insert(flight);

    travelers_t::insert_row(1, "Jean-Luc", "Picard", 101, 0, "basic");
    travelers_t::insert_row(2, "William", "Riker", 202, 0, "basic");
    travelers_t::insert_row(3, "Beverly", "Crusher", 303, 0, "basic");
    travelers_t::insert_row(4, "Deanna", "Troi", 404, 0, "basic");
    travelers_t::insert_row(5, "Worf", "", 505, 0, "basic");
    travelers_t::insert_row(6, "Geordi", "La Forge", 606, 0, "basic");
    travelers_t::insert_row(7, "Data", "", 707, 0, "basic");

/*
    auto trip = trips_t::insert_row(1, 1, "Jean-Luc's trip", 0, 0);
    airport_ohare.departure_trips().insert(trip);
    airport_seattle.arrival_trips().insert(trip);
*/

    tx.commit();
}

void dump_db()
{
    std::cout << "\n";
    std::cout << "-----------------------------------------\n";
    begin_transaction();
    for (auto ap : airplanes_t::list())
    {
        std::printf("%d\n", (int)ap.tail_number());
    }
    commit_transaction();
    std::cout << "-----------------------------------------\n";
    std::cout << "\n" << std::endl;
}

int main()
{
    gaia::system::initialize();

    std::cout << "-----------------------------------------\n";
    std::cout << "Frequent Flyer\n\n";
    std::cout << "No chickens or puppies were harmed in the\n";
    std::cout << "development or presentation of this demo.\n";
    std::cout << ";-) \n";
    std::cout << "-----------------------------------------\n";

    dump_db();
    init_storage();
    dump_db();
    gaia::system::shutdown();
}
