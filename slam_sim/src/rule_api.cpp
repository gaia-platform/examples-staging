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
#include "globals.hpp"
#include "occupancy.hpp"
#include "slam_sim.hpp"

namespace slam_sim
{

using std::string;

using gaia::slam::edges_t;
using gaia::slam::ego_t;
using gaia::slam::graphs_t;
using gaia::slam::positions_t;
using gaia::slam::vertices_t;

using gaia::slam::area_map_t;
using gaia::slam::destination_t;
using gaia::slam::error_corrections_t;
using gaia::slam::observed_area_t;

using gaia::slam::area_map_writer;
using gaia::slam::destination_writer;

// Determine if a new graph optimization is necessary.
// In a live example, this function would apply logic to determine if
//  enough data has been collected (e.g., new range data, new closures,
//  etc.) to justify performing another map optimization. For now, say 
//  an optiimzation is required every X vertices.
bool optimization_required()
{
    return true;
//    static int32_t ctr = 0;
//    if (ctr < 4)
//    {
//        return true;
//    }
//    else if (ctr < 8)
//    {
//        return (ctr & 1) == 0;
//    }
//    else if (ctr < 16)
//    {
//        return (ctr & 3) == 0;
//    }
//    else if (ctr < 32)
//    {
//        return (ctr & 7) == 0;
//    }
//    else
//    {
//        return (ctr & 15) == 0;
//    }
//    return false;
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
bool need_to_extend_map(positions_t& pos, observed_area_t& bounds)
{
    float right_meters = ceilf(pos.x_meters() + c_range_sensor_max_meters);
    float top_meters = ceilf(pos.y_meters() + c_range_sensor_max_meters);
    float left_meters = floorf(pos.x_meters() - c_range_sensor_max_meters);
    float bottom_meters = floorf(pos.y_meters() - c_range_sensor_max_meters);
    if (right_meters > bounds.right_meters())
    {
        return true;
    }
    if (left_meters < bounds.left_meters())
    {
        return true;
    }
    if (top_meters > bounds.top_meters())
    {
        return true;
    }
    if (bottom_meters < bounds.bottom_meters())
    {
        return true;
    }
    // Else, sensor range doesn't exceed map boundaries.
    return false;
}


static void build_map(area_map_t& am)
{
    static int32_t ctr = 0;
    occupancy_grid_t map(am);
    // There are C++ ways to format a number to a string (e.g.,
    //  fmt::format or std::stringstream, but the C approach is
    //  nice in its simplicity.
// TODO get timestamp to use for name
    char fname[256];
    sprintf(fname, "map_%03d.pnm", ctr++);
    gaia_log::app().info("Building map {}", fname);
    map.export_as_pnm(fname);
}


void export_map_to_file()
{
    bool existing_transaction = false;
    if (gaia::db::is_transaction_open()) 
    {
        existing_transaction = true;
    }
    if (!existing_transaction)
    {
        gaia::db::begin_transaction();
    }
    ego_t ego = *(ego_t::list().begin());
    area_map_t& am = *(area_map_t::list().begin());
    build_map(am);
    if (!existing_transaction)
    {
        gaia::db::commit_transaction();
    }
}


void build_area_map(destination_t& dest, area_map_t& am, 
    observed_area_t& bounds)
{
printf("Build area map\n");
    // Each time we build an area map 
    occupancy_grid_t area_map(am, bounds);
    for (graphs_t& g: graphs_t::list())
    {
//printf("Applying sensor data from graph %d\n", g.id());
        for (vertices_t& v: g.vertices())
        {
            gaia_log::app().info("Pulling sensor data from {}:{}", 
                g.id(), v.id());
//printf("    Sensor data from %d::%d\n", g.id(), v.id());
            area_map.apply_sensor_data(v);
        }
    }
//area_map.count_bounds();
    // Build paths to destination
    world_coordinate_t pos = {
        .x_meters = dest.x_meters(),
        .y_meters = dest.y_meters()
    };
    area_map.trace_routes(pos);
    // Update blob ID in database
    area_map_writer writer = am.writer();
    writer.blob_id = area_map.get_blob_id();
    writer.update_row();
area_map.count_bounds();
}

////////////////////////////////////////////////////////////////////////

void full_stop()
{
    // TODO Take observation.
    // Stop moving.
    g_running = false;
    // TODO Need way to decide when to start running again.
}


bool reassess_destination()
{
    // Determine if it's time to change destinations.
    // If w/in X meters of destination, select new one. It's OK
    //  if destination is in unexplored area.
    destination_t& dest = *(destination_t::list().begin());
    float dx = g_position.x_meters - dest.x_meters();
    float dy = g_position.y_meters - dest.y_meters();
    float dist_meters_2 = dx*dx + dy*dy;
    const float close_enough_radius_2 = c_destination_radius_meters
        * c_destination_radius_meters;
    if (dist_meters_2 < close_enough_radius_2)
    {
        // Select next destination.
        world_coordinate_t new_dest = g_destinations[g_next_destination++];
printf("Next destination: %.1f,%.1f\n", new_dest.x_meters, new_dest.y_meters);
        if (g_next_destination >= g_destinations.size())
        {
            g_next_destination = 0;
        }
        destination_writer w = dest.writer();
        w.x_meters = new_dest.x_meters;
        w.y_meters = new_dest.y_meters;
        w.update_row();
        g_now += 1.0;
        return true;
    }
    return false;
}

} // namespace slam_sim

