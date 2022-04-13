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

#include "constants.hpp"
#include "occupancy.hpp"
#include "slam_sim.hpp"

namespace slam_sim
{

using std::string;

using gaia::slam::destination_t;
using gaia::slam::edges_t;
using gaia::slam::ego_t;
using gaia::slam::error_corrections_t;
using gaia::slam::graphs_t;
using gaia::slam::observed_area_t;
using gaia::slam::vertices_t;
using gaia::slam::area_map_t;
using gaia::slam::working_map_t;


// Determine if a new graph optimization is necessary.
// In a live example, this function would apply logic to determine if
//  enough data has been collected (e.g., new range data, new closures,
//  etc.) to justify performing another map optimization. For now, say 
//  an optiimzation is required every X vertices.
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
        gaia_log::app().info("Edge connects vertices {} and {}",
            e.src().id(), e.dest().id());
    }
    // By vertices:
    for (vertices_t& o: vertices_t::list())
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

// Determines if sensor data extends beyond present area map and returns
//  'true' if so.
bool need_to_extend_map()
{
    assert(1 == 0);
    return false;
}


static void build_map(const graphs_t& g, const world_coordinate_t& bottom_left,
    float width_meters, float height_meters)
{
    static int32_t ctr = 0;
    occupancy_grid_t map(c_standard_map_node_width_meters, bottom_left,
        width_meters, height_meters);
    // Iterate through observations in this graph and build a map.
    for (const vertices_t& o: g.vertices())
    {
        gaia_log::app().info("Applying sensor data from vertex {}", 
            o.id());
        map.apply_sensor_data(o);
    }
    // There are C++ ways to format a number to a string (e.g.,
    //  fmt::format or std::stringstream, but the C approach is
    //  nice in its simplicity.
// TODO get timestamp to use for name
    char fname[256];
    sprintf(fname, "map_%03d.pgm", ctr++);
    gaia_log::app().info("Building map {}", fname);
    map.export_as_pnm(fname);
}


void export_map_to_file()
{
    gaia::db::begin_transaction();
    ego_t ego = *(ego_t::list().begin());
    area_map_t& am = *(area_map_t::list().begin());
    world_coordinate_t bottom_left = {
        .x_meters = am.left_meters(),
        .y_meters = am.bottom_meters()
    };
    float width_meters = am.right_meters() - am.left_meters();
    float height_meters = am.top_meters() - am.bottom_meters();
    build_map(ego.current_graph(), bottom_left, width_meters, height_meters);
    gaia::db::commit_transaction();
}


} // namespace slam_sim

