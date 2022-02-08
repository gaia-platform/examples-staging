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
    for (auto obj_it = T_table::list().begin();
        obj_it != T_table::list().end();)
    {
        auto next_obj_it = obj_it++;
        next_obj_it->delete_row();
    }
}

uint64_t get_time_millis()
{
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void remove_all_data()
{
    delete_all_rows<segments_t>();
    delete_all_rows<trips_t>();
    delete_all_rows<travelers_t>();
    delete_all_rows<flights_t>();
    delete_all_rows<routes_t>();
    delete_all_rows<airplanes_t>();
    delete_all_rows<airports_t>();
}

void init_storage()
{
    begin_transaction();

    remove_all_data();

    airports_t::insert_row("SEA");
    airports_t::insert_row("LAX");
    airports_t::insert_row("ORD");
    airports_t::insert_row("BOS");

    airplanes_t::insert_row(10, 10000);
    airplanes_t::insert_row(20, 20000);
    airplanes_t::insert_row(30, 30000);
    airplanes_t::insert_row(40, 40000);

    auto route_seattle_la = routes_t::get(routes_t::insert_row(960, "SEA", "LAX"));
    auto route_la_seattle = routes_t::get(routes_t::insert_row(960, "LAX", "SEA"));
    auto route_la_ohare = routes_t::get(routes_t::insert_row(1745, "LAX", "ORD"));
    auto route_ohare_la = routes_t::get(routes_t::insert_row(1745, "ORD", "LAX"));
    auto route_ohare_seattle = routes_t::get(routes_t::insert_row(1716, "ORD", "SEA"));
    auto route_ohare_boston = routes_t::get(routes_t::insert_row(867, "ORD", "BOS"));
    auto route_boston_seattle = routes_t::get(routes_t::insert_row(2485, "BOS", "SEA"));

    route_seattle_la.flights().insert(flights_t::insert_row(1, 0, 0, "scheduled", 10, 0));
    route_la_ohare.flights().insert(flights_t::insert_row(2, 0, 0, "scheduled", 20, 0));
    route_ohare_boston.flights().insert(flights_t::insert_row(3, 0, 0, "scheduled", 20, 0));
    route_boston_seattle.flights().insert(flights_t::insert_row(4, 0, 0, "scheduled", 20, 0));
    route_ohare_seattle.flights().insert(flights_t::insert_row(5, 0, 0, "scheduled", 30, 0));
    route_seattle_la.flights().insert(flights_t::insert_row(6, 0, 0, "scheduled", 30, 0));
    route_la_seattle.flights().insert(flights_t::insert_row(7, 0, 0, "scheduled", 30, 0));
    route_ohare_la.flights().insert(flights_t::insert_row(8, 0, 0, "scheduled", 40, 0));
    route_la_ohare.flights().insert(flights_t::insert_row(9, 0, 0, "scheduled", 40, 0));

    travelers_t::insert_row(1, "Jean-Luc", "Picard", 0, "basic");
    travelers_t::insert_row(2, "Worf", "", 0, "basic");
    travelers_t::insert_row(3, "Data", "", 0, "basic");

    trips_t::insert_row(2, 1, "Jean-Luc's trip 2", 0, 0, "SEA", "LAX");
    segments_t::insert_row(1, 6, 2);

    trips_t::insert_row(1, 1, "Jean-Luc's trip 1", 0, 0, "ORD", "SEA");
    segments_t::insert_row(2, 7, 1);
    segments_t::insert_row(1, 8, 1);

    trips_t::insert_row(3, 2, "Warf's trip", 0, 0, "LAX", "SEA");
    segments_t::insert_row(1, 7, 3);

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
            << std::right << std::setw(26) << route.departure_airport().airport_code() << " to "
            << std::left << std::setw(35) << route.arrival_airport().airport_code() << "|\n";
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
            std::cout << "    | Trip to " << std::left << std::setw(51) << trip.arrival_airport().airport_code() << "|\n";
            for (auto segment : trip.segments())
            {
                std::cout << "   "
                    << " | Segment: " << std::setw(4) << (int)segment.segment_order() << "| "
                    << std::right << std::setw(12) << segment.flight().route().departure_airport().airport_code() << " to "
                    << std::left << std::setw(12) << segment.flight().route().arrival_airport().airport_code()
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
                std::cout << "\n";

                begin_transaction();
                auto flight_iter = flights_t::list().where(flights_t::expr::flight_number == flight_number).begin();
                if (flight_iter != flights_t::list().end()
                    && (strcmp(flight_iter->flight_status(), "scheduled") == 0))
                {
                    auto flight_w = flight_iter->writer();
                    flight_w.flight_status = "landed";
                    flight_w.flight_miles = flight_iter->route().route_miles();
                    flight_w.update_row();
                    std::cout << "Flight miles and status updated.\n";
                }
                else
                {
                    std::cout << "Flight already landed.\n";
                }
                commit_transaction();
                std::cout << "Press enter to continue.";
                read_input();
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
