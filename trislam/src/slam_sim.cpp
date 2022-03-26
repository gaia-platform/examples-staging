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

using std::string;


namespace slam_sim
{

constexpr double RULE_WAIT_SEC = 0.1;

// If there is a transaction conflict when creating and observation then
//  retry again a finite number of times. If those fail then wait for
//  the next observation and try again.
constexpr uint32_t NUM_OBSERVATION_TXN_RETRIES = 2;

static const char* s_path_file = NULL;

void move_bot_along_path()
{
    // Path file contain points that define the path. The sequence of 
    //  points define segments.
    assert(s_path_file);
    coord_list_t path(s_path_file);
    map_coord_t prev = path[0];
    for (map_coord_t& latest: path)
    {
        // Create observation at this location.
        //  observation to be made.
        for (uint32_t i=0; i<NUM_OBSERVATION_TXN_RETRIES; i++)
        {
            try
            {
                gaia::db::begin_transaction();
                create_observation(prev, latest);
                gaia::db::commit_transaction();
            }
            catch (gaia::db::transaction_update_conflict&)
            {
                // Take a brief nap and try again.
                usleep(1000);
            }
        }
        // wait
        usleep((uint32_t) (RULE_WAIT_SEC * 1.0e-6));
        prev = latest;
    }
    // Output final map.
    build_map();
}


template <class T>
T get_single_record()
{
    return *(T::list().begin());
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

// Clean up the database.
void clean_db()
{
    gaia::db::begin_transaction();
    remove_all_rows<gaia::slam::ego_t>();
    remove_all_rows<gaia::slam::observed_area_t>();
    remove_all_rows<gaia::slam::latest_observation_t>();

    remove_all_rows<gaia::slam::graphs_t>();
    remove_all_rows<gaia::slam::edges_t>();
    remove_all_rows<gaia::slam::error_corrections_t>();
    remove_all_rows<gaia::slam::observations_t>();
    remove_all_rows<gaia::slam::positions_t>();
    remove_all_rows<gaia::slam::movements_t>();
    remove_all_rows<gaia::slam::range_data_t>();
    gaia::db::commit_transaction();
}


static void usage(int, char** argv)
{
    printf("Gaia SLAM simulator\n\n");
    printf("Usage: %s -m <file path> -p <path file>\n", argv[0]);
    printf("  where\n");
    printf("    -m    name of json file describing the world (e.g., "
        "data/map.json)\n");
    printf("    -p    name of json file listing path bot takes (e.g., "
        "data/path.txt\n");
    exit(1);
}


void parse_command_line(int argc, char** argv)
{
    int opt;
    const char* map_file = NULL;
    while ((opt = getopt(argc, argv, "hm:p:")) != -1)
    {
        switch(opt)
        {
            case 'm':
                map_file = optarg;
                break;
            case 'p':
                s_path_file = optarg;
                break;
            case 'h':
                usage(argc, argv);
                break;
            default:
                usage(argc, argv);
        }
    }
    if ((map_file == NULL) || (s_path_file == NULL))
    {
        usage(argc, argv);
    }
    gaia_log::app().info("Loading world map {}", map_file);
    gaia_log::app().info("Using path file {}", s_path_file);
    load_world_map(map_file);
}


void init_sim()
{
    // Seed database and then create first path.
    // Seeding function manages its own transaction.
    gaia_log::app().info("Seeding the database...");
    seed_database();
}

} // namespace slam_sim;


int main(int argc, char** argv)
{
    gaia::system::initialize();

    slam_sim::parse_command_line(argc, argv);

    slam_sim::clean_db();
    slam_sim::init_sim();
    slam_sim::move_bot_along_path();

    gaia::system::shutdown();
}

