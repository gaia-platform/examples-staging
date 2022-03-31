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

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>

#include "gaia_slam.h"

#include "occupancy.hpp"
#include "slam_sim.hpp"

namespace slam_sim
{

using std::string;

using gaia::slam::edges_t;
using gaia::slam::ego_t;
using gaia::slam::error_corrections_t;
using gaia::slam::graphs_t;
using gaia::slam::observed_area_t;
using gaia::slam::observations_t;


// Determine if a new graph optimization is necessary.
// In a live example, this function would apply logic to determine if
//  enough data has been collected (e.g., new range data, new closures,
//  etc.) to justify performing another map optimization. For now, say 
//  an optiimzation is required every X observations.
bool optimization_required()
{
    static int32_t ctr = 0;
    const int32_t MAP_INTERVAL = 10;
    if ((++ctr % MAP_INTERVAL) == 0)
    {
        return true;
    }
    return false;
}


// Optimze the graph.
// In a live example this would iterate through all edges/nodes and
//  do some math wizardry to reduce positional error of the nodes. For
//  now, iterate though the nodes/edges to demonstrate how that works
//  and just print error to the log.
void optimize_graph(graphs_t& graph)
{
    // Map optimization logic goes here. Here are some ways to iterate
    //  through the data.
    gaia_log::app().info("Stub function to optimize graph {}", graph.id());
    // By edges:
    for (edges_t& e: edges_t::list())
    {
        gaia_log::app().info("Edge connects observations {} and {}",
            e.src().id(), e.dest().id());
    }
    // By observations:
    for (observations_t& o: observations_t::list())
    {
        gaia_log::app().info("Obervation {} connects to:", o.id());
        for (edges_t& e: o.in_edges())
        {
            gaia_log::app().info("  (in) obervation {}", e.src_id());
        }
        for (edges_t& e: o.out_edges())
        {
            gaia_log::app().info("  (out) obervation {}", e.dest_id());
        }
    }
    // Create error correction record. This serves to store error correction
    //  data, if necessary, as well as trigger a rule event.
    static uint32_t next_ec_id = 1;
    error_corrections_t::insert_row(
        next_ec_id,         // id
        graph.id()          // graph_id
    );
    next_ec_id++;
}


////////////////////////////////////////////////////////////////////////
// Map building

static void write_map_to_file(occupancy_grid_t& map)
{
    static int32_t ctr = 0;
    // There are C++ ways to format a number to a string (e.g.,
    //  fmt::format or std::stringstream, but the C approach is
    //  nice in its simplicity.
    char fname[256];
    sprintf(fname, "map_%03d.pgm", ctr++);
    gaia_log::app().info("Building map {}", fname);
    map.export_as_pnm(fname);
}


void build_map()
{
    gaia::db::begin_transaction();
    ego_t ego = *(ego_t::list().begin());
    observed_area_t& area = *(observed_area_t::list().begin());
    build_map(ego.current_graph(), area);
    gaia::db::commit_transaction();
}


void build_map(const graphs_t& g, const observed_area_t& bounds)
{
    world_coordinate_t top_left = {
        .x_meters = bounds.left_meters(),
        .y_meters = bounds.top_meters()
    };
    world_coordinate_t bottom_right = {
        .x_meters = bounds.right_meters(),
        .y_meters = bounds.bottom_meters()
    };
    occupancy_grid_t map(c_map_node_width_meters, top_left, bottom_right);
    // Iterate through observations in this graph and build a map.
    for (const observations_t& o: g.observations())
    {
        gaia_log::app().info("Applying sensor data from observation {}", 
            o.id());
        map.apply_sensor_data(o);
    }
    write_map_to_file(map);
}

} // namespace slam_sim

