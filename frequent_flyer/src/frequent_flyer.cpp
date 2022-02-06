/////////////////////////////////////////////
// Copyright (c) 2022 Gaia Platform LLC
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
/////////////////////////////////////////////

#include <chrono>
#include <iostream>
#include <iomanip>

#include "gaia/logger.hpp"
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

    for (auto flight = *flights_t::list().begin();
        flight; flight = *flights_t::list().begin())
    {
        flight.delete_row();
    }

    delete_all_rows<travelers_t>();
}

void init_storage()
{
    begin_transaction();

    remove_all_data();

    auto route_seattle_la = routes_t::get(routes_t::insert_row(960));
    auto route_la_seattle = routes_t::get(routes_t::insert_row(960));
    auto route_la_ohare = routes_t::get(routes_t::insert_row(1745));
    auto route_ohare_la = routes_t::get(routes_t::insert_row(1745));
    auto route_ohare_seattle = routes_t::get(routes_t::insert_row(1716));
    auto route_ohare_boston = routes_t::get(routes_t::insert_row(867));
    auto route_boston_seattle = routes_t::get(routes_t::insert_row(2485));

    auto airplane_1 = airplanes_t::get(airplanes_t::insert_row(1, 10000));
    auto airplane_2 = airplanes_t::get(airplanes_t::insert_row(2, 20000));
    auto airplane_3 = airplanes_t::get(airplanes_t::insert_row(3, 30000));
    auto airplane_4 = airplanes_t::get(airplanes_t::insert_row(4, 40000));

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

    auto flight = flights_t::get(flights_t::insert_row(1, 1, 0, 0, "scheduled", 0));
    airplane_1.flights().insert(flight);
    route_seattle_la.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(2, 2, 0, 0, "scheduled", 0));
    airplane_2.flights().insert(flight);
    route_la_ohare.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(3, 3, 0, 0, "scheduled", 0));
    airplane_2.flights().insert(flight);
    route_ohare_boston.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(4, 4, 0, 0, "scheduled", 0));
    airplane_2.flights().insert(flight);
    route_boston_seattle.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(5, 5, 0, 0, "scheduled", 0));
    airplane_3.flights().insert(flight);
    route_ohare_seattle.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(6, 6, 0, 0, "scheduled", 0));
    airplane_3.flights().insert(flight);
    route_seattle_la.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(7, 7, 0, 0, "scheduled", 0));
    airplane_3.flights().insert(flight);
    route_la_seattle.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(8, 8, 0, 0, "scheduled", 0));
    airplane_4.flights().insert(flight);
    route_ohare_la.flights().insert(flight);

    flight = flights_t::get(flights_t::insert_row(9, 9, 0, 0, "scheduled", 0));
    airplane_4.flights().insert(flight);
    route_la_ohare.flights().insert(flight);

    travelers_t::insert_row(1, "Jean-Luc", "Picard", 0, "basic");
    travelers_t::insert_row(2, "Worf", "", 0, "basic");

    auto trip = trips_t::insert_row(2, 1, "Jean-Luc's trip 2", 0, 0);
    airport_seattle.departure_trips().insert(trip);
    airport_la.arrival_trips().insert(trip);
    auto segment = segments_t::insert_row(1, 6, 2);

    trip = trips_t::insert_row(1, 1, "Jean-Luc's trip 1", 0, 0);
    airport_ohare.departure_trips().insert(trip);
    airport_seattle.arrival_trips().insert(trip);
    segment = segments_t::insert_row(2, 7, 1);
    segment = segments_t::insert_row(1, 8, 1);

    trip = trips_t::insert_row(3, 2, "Warf's trip", 0, 0);
    airport_la.departure_trips().insert(trip);
    airport_seattle.arrival_trips().insert(trip);
    segment = segments_t::insert_row(1, 7, 3);

    commit_transaction();
}

void dump_db()
{
    begin_transaction();
    std::cout << "\n+========+\n";
    std::cout << "| Routes |\n";
    std::cout << "+======================================================================\n";
    for (auto route : routes_t::list())
    {
        std::cout << "    |"
            << std::right << std::setw(26) << route.departure_airport().city() << " to "
            << std::left << std::setw(35) << route.arrival_airport().city() << "|\n";
        for (auto flight : route.flights())
        {
            std::cout << "    +--------------+----------------------+---------------------------+\n";
            std::cout << "   "
                << " | Flight: " << std::setw(4) << flight.flight_number()
                << " | Flight miles: " << std::setw(6) << flight.flight_miles()
                << " | Flight status: " << std::setw(11) << flight.flight_status() << "|\n";
        }
        std::cout << "    ===================================================================\n";
    }

    std::cout << "\n+===========+\n";
    std::cout << "| Travelers |\n";
    std::cout << "+=================================================================\n";
    for (auto traveler : travelers_t::list())
    {
        std::cout << "   "
            << " | " << std::setw(9) << traveler.first_name()
            << " | Lifetime miles: " << std::setw(7) << traveler.lifetime_miles()
            << " | Member level: " << std::setw(7) << traveler.member_level() << "|\n";
        for (auto trip : traveler.trips())
        {
            std::cout << "    +------------------------------------------------------------+\n";
            std::cout << "    | Trip to " << std::left << std::setw(51) << trip.arrival_airport().city() << "|\n";
            for (auto segment : trip.segments())
            {
                std::cout << "   "
                    << " | Segment: " << std::setw(4) << (int)segment.segment_order() << "| "
                    << std::setw(12) << segment.flight().route().departure_airport().city() << " to "
                    << std::setw(12) << segment.flight().route().arrival_airport().city()
                    << "  | Flight: " << std::setw(4) << segment.flight().flight_number() << "|\n";
            }
        }
        std::cout << "    ==============================================================\n";
    }

    bool first_flight = true;
    std::cout << "\n+=========+\n";
    std::cout << "| Flights |\n";
    std::cout << "+=====================================================================\n";
    for (auto flight : flights_t::list())
    {
        if (!first_flight)
        {
            std::cout << "    +-------------+----------------------+---------------------------+\n";
        }
        first_flight = false;
        std::cout << "   "
            << " | Flight: " << std::setw(3) << flight.flight_number()
            << " | Flight miles: " << std::setw(6) << flight.flight_miles()
            << " | Flight status: " << std::setw(11) << flight.flight_status() << "|\n";
    }
    std::cout << "    ==================================================================\n";
    commit_transaction();
}

class simulation_t
{
public:
    static constexpr char c_cmd_quit = 'q';
    const char* c_wrong_input = "Wrong input.";

    bool read_input()
    {
        std::getline(std::cin, m_input);
        return !std::cin.eof();
    }

    bool handle_main()
    {
        dump_db();
        std::cout << "\n";
        std::cout << "Enter flight number to complete or " << c_cmd_quit << " to quit > ";

        if (!read_input())
        {
            return false;
        }

        if (m_input.size() == 1)
        {
            if (m_input[0] == c_cmd_quit)
            {
                std::cout << "Exiting..." << std::endl;
                return false;
            }
            if (m_input[0] >= '0' && m_input[0] <= '9')
            {
                uint32_t flight_number = m_input[0] - '0';
                begin_transaction();
                auto flight_iter = flights_t::list().where(flights_t::expr::flight_number == flight_number).begin();
                if (flight_iter != flights_t::list().end()
                    && (strcmp(flight_iter->flight_status(), "scheduled") == 0))
                {
                    auto flight_w = flight_iter->writer();
                    flight_w.flight_status = "landed";
                    flight_w.flight_miles = flight_iter->route().route_miles();
                    flight_w.update_row();
                }
                commit_transaction();
                return true;
            }
        }
        wrong_input();
        return true;
    }

    int run()
    {
        bool has_input = true;
        while (has_input)
        {
            has_input = handle_main();
        }
        return EXIT_SUCCESS;
    }

private:
    std::string m_input;

    void wrong_input()
    {
        std::cerr << c_wrong_input << std::endl;
    };
};

int main()
{
    simulation_t sim;
    gaia::system::initialize();

    std::cout << "+================+\n";
    std::cout << "| Frequent Flyer |\n";
    std::cout << "+================+\n";

    init_storage();
    sim.run();
    gaia::system::shutdown();
}
