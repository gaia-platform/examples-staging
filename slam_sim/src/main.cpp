////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <unistd.h>
#include <getopt.h>

#include <chrono>

#include <thread>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>
#include <gaia/system.hpp>

#include "slam_sim.hpp"

using std::this_thread::sleep_for;

namespace slam_sim
{

constexpr uint32_t c_rule_wait_millis = 100;

int32_t g_quit = 0;

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
}


static void usage(int argc, char** argv)
{
    printf("Gaia SLAM simulator\n\n");
    printf("Usage: %s -m <map.json>\n", argv[0]);
    exit(1);
}


void parse_command_line(int argc, char** argv)
{
    int opt;
    const char* map_file = NULL;
    while ((opt = getopt(argc, argv, "hm:")) != -1)
    {
        switch(opt)
        {
            case 'm':
                map_file = optarg;
                break;
            case 'h':
                usage(argc, argv);
                break;
            default:
                usage(argc, argv);
        }
    }
    if (map_file == NULL)
    {
        usage(argc, argv);
    }
    load_world_map(map_file);
}


void init_sim()
{
    // Seed database and then create first path.
    // Seeding function manages its own transaction.
    gaia_log::app().info("Seeding the database");
    seed_database();

    gaia_log::app().info("Creating initial path");
    gaia::db::begin_transaction();
    create_new_path();
    gaia::db::commit_transaction();
}

} // namespace slam_sim;


int main(int argc, char** argv)
{
    slam_sim::parse_command_line(argc, argv);

    gaia::system::initialize();

    // We explicitly handle the transactions with begin_transaction() and commit_transaction()
    // to trigger the rules.
    gaia::db::begin_transaction();
    slam_sim::clear_data();
    gaia::db::commit_transaction();

    gaia_log::app().info("=== Creates a new Graph and observe the corresponding rules triggered ===");

    slam_sim::init_sim();
    slam_sim::wait_for_rules();

    gaia::system::shutdown();
}
