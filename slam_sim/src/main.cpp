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

using namespace gaia::slam;

using std::this_thread::sleep_for;

namespace slam_sim
{

constexpr uint32_t c_rule_wait_millis = 100;

int32_t g_quit = 0;

static float g_initial_x_meters = -1.0;
static float g_initial_y_meters = -1.0;

/**
 * Wait an arbitrary amount of time for rule execution to terminate.
 * Rules are triggered after commit and can take some time to fully execute.
 */
void wait_for_rules()
{
    while (g_quit == 0)
    {
        sleep_for(std::chrono::milliseconds(c_rule_wait_millis));
    }
}


template <typename T_type>
void clear_table()
{
    for (auto obj_it = T_type::list().begin(); obj_it != T_type::list().end(); )
    {
        auto current_obj_it = obj_it++;
        current_obj_it->delete_row();
    }
}


void clear_data()
{
    clear_table<edges_t>();
    clear_table<landmark_sightings_t>();
    clear_table<landmarks_t>();
    clear_table<paths_t>();
    clear_table<observations_t>();
    clear_table<collision_event_t>();
    clear_table<collision_event_t>();
    clear_table<pending_destination_t>();
    clear_table<error_correction_t>();
    clear_table<estimated_position_t>();
    clear_table<destination_t>();
    clear_table<working_map_t>();
    clear_table<local_map_t>();
    clear_table<area_map_t>();
    clear_table<ego_t>();
    clear_table<sim_position_offset_t>();
}


static void usage(int, char** argv)
{
    printf("Gaia SLAM simulator\n\n");
    printf("Usage: %s -m <file path> -x <coord> -y <coord>\n", argv[0]);
    printf("  where\n");
    printf("    -m    the json file describing the world (e.g., "
        "data/map.json)\n");
    printf("    -x    starting X coordinate of bot (must be >0)\n");
    printf("    -y    starting Y coordinate of bot (must be >0)\n");
    exit(1);
}


void parse_command_line(int argc, char** argv)
{
    int opt;
    const char* map_file = NULL;
    double g_initial_x_meters = -1.0;
    double g_initial_y_meters = -1.0;
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
                break;
            case 'h':
                usage(argc, argv);
                break;
            default:
                usage(argc, argv);
        }
    }
    if ((map_file == NULL) || (g_initial_x_meters <= 0.0) || 
        (g_initial_y_meters < 0.0))
    {
        usage(argc, argv);
    }
    load_world_map(map_file);
}


void init_sim()
{
    // Seed database and then create first path.
    // Seeding function manages its own transaction.
    gaia_log::app().info("Seeding the database...");
    seed_database(g_initial_x_meters, g_initial_y_meters);

    gaia_log::app().info("Creating initial path...");
    gaia::db::begin_transaction();
    create_new_path();
    gaia::db::commit_transaction();
}

} // namespace slam_sim;


int main(int argc, char** argv)
{
    slam_sim::parse_command_line(argc, argv);

    gaia::system::initialize();

    // We explicitly handle the transactions with begin_transaction() 
    //  and commit_transaction() to trigger the rules.
    gaia::db::begin_transaction();
    slam_sim::clear_data();
    gaia::db::commit_transaction();

    gaia_log::app().info("Starting SLAM simulation...");

    slam_sim::init_sim();
    slam_sim::wait_for_rules();

    gaia::system::shutdown();
}
