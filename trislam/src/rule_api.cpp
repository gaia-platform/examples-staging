////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Primary API for SLAM-related rules.
//
// Maps are generated from sensor data stored in the current graph.
//
////////////////////////////////////////////////////////////////////////

#include <gaia/logger.hpp>

#include "slam_sim.hpp"

namespace slam_sim
{

void move_bot_along_path(string path_file)
{
    // Path file contain points that define the path. The sequence of 
    //  points define segments.
    coord_list_t path(path_file);
    for (map_coord_t& mc: path)
    {
        // TODO create observation at this location


        // Wait for a short while before moving ahead.
        usleep(50000);
    }
    
    // TODO 'optimize' graph and output final map
}


bool optimization_required()
{
    // Force an optimization every X observations
    static int32_t ctr = 0;
    const int32_t MAP_INTERVAL = 10;
    if ((++ctr % MAP_INTERVAL) == 0)
    {
        return true;
    }
    return false;
}

void optimize_graph(graphs_t& graph)
{
    // Map optimization logic goes here. Here are some ways to iterate
    //  through the data.
    // By edges:
    for (edges_t& e: edges_t::list())
    {
        gaia_log::app()info("Edge {} connects observations {} and {}",
            e.id(), e.src().id(), e.dest().id());
    }

    // By observations:
    for (observations_t& o: observations_t::list())
    {
        gaia_log::app()info("Obervation {} connects to:", o.id());
        for (edges_t& e: o.in_edges())
        {
            gaia_log::app()info("  (in) obervation {} via edge {}:", e.src(), 
                e.id());
        }
        for (edges_t& e: o.out_edges())
        {
            gaia_log::app()info("  (out) obervation {} via edge {}:", 
                e.dest(), e.id());
        }
    }

    // TODO Create error correction record 
}


////////////////////////////////////////////////////////////////////////
// Map building

static void write_map_to_file(observation_grid_t& map)
{
    static int32_t ctr = 0;
    string fname("map_" + std::to_string(ctr) + ".pgm");
    gaia_log::app().info("Building map {}", fname);
    map.export_as_pnm(fname);
    ctr++;
}


void build_map()
{
    ego_t ego = *(ego_t::list().begin());
    observed_area_t& area = *(observed_area_t::list().begin());
    build_map(ego.current_graph(), area);
}


void build_map(graphs_t& g, observed_area_t& bounds)
{
    world_coordinate_t top_left = {
        .x_meters = bounds.left_meters(),
        .y_meters = bounds.top_meters()
    };
    world_coordinate_t bottom_right = {
        .x_meters = bounds.right_meters(),
        .y_meters = bounds.bottom_meters()
    };
    observation_grid_t map(MAP_NODE_WIDTH_METERS, top_left, bottom_right);
    // Iterate through observations in this graph and build a map.
    for (observations_t& o: g.observations())
    {
        gaia_log::app()info("Applying sensor data from observation {}", 
            o.id());
        map.apply_sensor_data(obs);
    }
    write_map_to_file(map);
}

} // namespace slam_sim

