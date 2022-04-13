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
using gaia::slam::vertices_t;

using gaia::slam::area_map_t;
using gaia::slam::destination_t;
using gaia::slam::error_corrections_t;
using gaia::slam::observed_area_t;
//using gaia::slam::working_map_t;

using gaia::slam::area_map_writer;
//using gaia::slam::working_map_writer;

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


void build_area_map(destination_t& dest, area_map_t& am, 
    observed_area_t& bounds)
{
    // Each time we build an area map 
    occupancy_grid_t area_map(am, bounds);
    for (graphs_t& g: graphs_t::list())
    {
        for (vertices_t& v: g.vertices())
        {
            gaia_log::app().info("Pulling sensor data from {}:{}", 
                g.id(), v.id());
            area_map.apply_sensor_data(v);
        }
    }
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
//    export_area_map();
}

////////////////////////////////////////////////////////////////////////
// YET TO FINISH

void build_working_map(occupancy_grid_t& working_map,
    destination_t& dest, area_map_t& am)
{
    occupancy_grid_t area_grid(am);
    // Set boundary conditions.
    // Iterate through boundary nodes of grid. Look up path values
    //  from area map.
    grid_size_t sz = working_map.get_grid_size();
    for (uint32_t y=0; y<sz.rows; y++)
    {
        for (uint32_t x=0; x<sz.cols; x++)
        {
            // For middle rows, only seed first and last column.
            if ((y != 0) && (y != sz.rows-1))
            {
                // In one of middle rows. Check if in first or last col.
                if ((x != 0) && (x != sz.cols-1))
                {
                    // Nope -- nothing to do here.
                    continue;
                }
            }
            // Get node from working map.
            grid_index_t idx = { .idx = x + y * sz.cols };
            map_node_t& local_node = working_map.get_node(idx);
            // Get corresponding node from area map.
            grid_coordinate_t xy = { .x=x, .y=y };
            world_coordinate_t pos = working_map.get_node_position(xy);
            map_node_t& world_node = 
                area_grid.get_node(pos.x_meters, pos.y_meters);
            local_node.path_cost = world_node.path_cost;
        }
    }
    // TODO Enter observation data since area map was generated.
    




    // Find paths toward destination.
    world_coordinate_t dest_coord = {
        .x_meters = dest.x_meters(),
        .y_meters = dest.y_meters()
    };
    area_grid.trace_routes(dest_coord);
}


void full_stop()
{
    // TODO Take observation and stop moving.
    // TODO Need way to decide when to move_toward_destination() again.
}


// Infrastructure has info on latest position so no need to query DB
//  (infra is what sent that info to the DB).

static void create_keyframe()
{
    // TODO add vertex to DB
}

void move_toward_destination(destination_t& dest, area_map_t& am)
{
    // Need position, area map, destination
    // Generate working map.
    world_coordinate_t bottom_left = {
        .x_meters = floorf(-c_range_sensor_max_meters),
        .y_meters = floorf(-c_range_sensor_max_meters)
    };
    float right_meters = ceilf(c_range_sensor_max_meters);
    float top_meters = ceilf(c_range_sensor_max_meters);
    float width_meters = right_meters - bottom_left.x_meters;
    float height_meters = top_meters - bottom_left.y_meters;
    occupancy_grid_t working_map(c_working_map_node_width_meters,
        bottom_left, width_meters, height_meters);
    build_working_map(working_map, dest, am);
    // Make several small steps.
    for (uint32_t i=0; i<c_num_steps_between_keyframes; i++)
    {
        // Get direction to head from map.
        map_node_t& node = working_map.get_node(g_position.x_meters, 
            g_position.y_meters);
        float heading_degs = node.direction_degs;
        // Move in that direction.
        float dist_meters = c_step_meters;
        float s, c;
        sincosf(c_deg_to_rad * heading_degs, &s, &c);
        float dx_meters = s * dist_meters;
        float dy_meters = c * dist_meters;
        g_position.x_meters += dx_meters;
        g_position.y_meters += dy_meters;
        gaia_log::app().info("Moving {},{} meters to {},{}", dx_meters,
            dy_meters, g_position.x_meters, g_position.y_meters);
    }
    create_keyframe();
}

bool reassess_destination()
{
    // TODO Determine if it's time to change destinations.
    return false;
}

} // namespace slam_sim

