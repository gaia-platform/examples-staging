////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>

#include <thread>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>
#include <gaia/system.hpp>

#include "gaia_slam.h"

#include "slam_sim.hpp"


namespace slam_sim
{

using std::this_thread::sleep_for;

static constexpr uint32_t c_rule_wait_millis = 100;


// Globals for this namespace
int32_t g_quit = 0;
float g_initial_x_meters = 0.0;
float g_initial_y_meters = 0.0;


/**
 * Wait for simulation to complete.
 */
void main_loop()
{
    // When the simulation completes it will set g_quit to 1. Then we can
    //  exit. In the meantime, the simulation is being handled by rules.
    while (g_quit == 0)
    {
        sleep_for(std::chrono::milliseconds(c_rule_wait_millis));
    }
}


template <class T_object>
void remove_all_rows()
{
    const bool force_disconnect_of_related_rows = true;
    for (auto obj_it = T_object::list().begin();
         obj_it != T_object::list().end();)
    {
        auto this_it = obj_it++;
        this_it->delete_row(force_disconnect_of_related_rows);
    }
}


void clean_db()
{
    gaia::db::begin_transaction();
    remove_all_rows<gaia::slam::ego_t>();
    remove_all_rows<gaia::slam::area_map_t>();
    remove_all_rows<gaia::slam::working_map_t>();
    remove_all_rows<gaia::slam::destination_t>();
    remove_all_rows<gaia::slam::observed_area_t>();
    remove_all_rows<gaia::slam::pending_destination_t>();
    remove_all_rows<gaia::slam::collision_event_t>();
    remove_all_rows<gaia::slam::graphs_t>();
    remove_all_rows<gaia::slam::observations_t>();
    remove_all_rows<gaia::slam::positions_t>();
    remove_all_rows<gaia::slam::movements_t>();
    remove_all_rows<gaia::slam::range_data_t>();
    remove_all_rows<gaia::slam::edges_t>();
    remove_all_rows<gaia::slam::error_corrections_t>();
    gaia::db::commit_transaction();
}


static void usage(int, char** argv)
{
    printf("Gaia SLAM simulator\n\n");
    printf("Usage: %s -m <file path> -x <coord> -y <coord>\n", argv[0]);
    printf("  where\n");
    printf("    -m    the json file describing the world (e.g., "
        "data/map.json)\n");
    printf("    -x    starting X coordinate of bot\n");
    printf("    -y    starting Y coordinate of bot\n");
    exit(1);
}


void parse_command_line(int argc, char** argv)
{
    int opt;
    const char* map_file = NULL;
    bool have_x = false;
    bool have_y = false;
    while ((opt = getopt(argc, argv, "hm:x:y:")) != -1)
    {
        switch(opt)
        {
            case 'm':
                map_file = optarg;
                break;
            case 'x':
                try
                {
                    g_initial_x_meters = std::stod(optarg, NULL);
                }
                catch (std::invalid_argument& e)
                {
                    usage(argc, argv);
                }
                have_x = true;
                break;
            case 'y':
                try
                {
                    g_initial_y_meters = std::stod(optarg, NULL);
                }
                catch (std::invalid_argument& e)
                {
                    usage(argc, argv);
                }
                have_y = true;
                break;
            case 'h':
                usage(argc, argv);
                break;
            default:
                usage(argc, argv);
        }
    }
    if ((map_file == NULL) || !have_x || !have_y)
    {
        usage(argc, argv);
    }
    gaia_log::app().info("Initial possition at {},{}", 
        g_initial_x_meters, g_initial_y_meters);
    gaia_log::app().info("Loading world map {}", map_file);
    load_world_map(map_file);
}


void init_sim()
{
    // Seed database and then create first path.
    // Seeding function manages its own transaction.
    gaia_log::app().info("Seeding the database...");
    seed_database(g_initial_x_meters, g_initial_y_meters);
}

} // namespace slam_sim;


int main(int argc, char** argv)
{
    gaia::system::initialize();

    slam_sim::parse_command_line(argc, argv);

    slam_sim::clean_db();
    slam_sim::init_sim();
    slam_sim::main_loop();

    gaia::system::shutdown();
}
