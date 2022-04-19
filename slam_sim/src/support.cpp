////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Supporting functions.
//
////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <math.h>

#include <iostream>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>

#include "constants.hpp"
#include "globals.hpp"
#include "slam_sim.hpp"
//#include "line_segment.hpp"


namespace slam_sim
{

static int32_t s_next_vertex_id = 1;

//
using gaia::common::gaia_id_t;
//
using gaia::slam::area_map_t;
using gaia::slam::edges_t;
using gaia::slam::destination_t;
using gaia::slam::ego_t;
using gaia::slam::graphs_t;
using gaia::slam::latest_observation_t;
using gaia::slam::movements_t;
using gaia::slam::observed_area_t;
using gaia::slam::positions_t;
using gaia::slam::range_data_t;
using gaia::slam::vertices_t;

using gaia::slam::latest_observation_writer;
using gaia::slam::observed_area_writer;


////////////////////////////////////////////////////////////////////////
// Background API that supports the contents of rule_api.cpp. Most
//  importantly, this means that the functions here are expected to 
//  be called from within an active transaction

static void update_observed_area(ego_t& ego)
{
    observed_area_t area = ego.world();
    bool change = false;
    // Left and right bounds.
    float left_edge   = area.left_meters();
    float right_edge  = area.right_meters();
    float max_range = c_range_sensor_max_meters;
    if (floor(g_position.x_meters - max_range) < left_edge)
    {
        left_edge = floor(g_position.x_meters - max_range);
        change = true;
    }
    if (ceil(g_position.x_meters + max_range) > right_edge)
    {
        right_edge = floor(g_position.x_meters + max_range);
        change = true;
    }
    // Top and bottom bounds.
    float bottom_edge = area.bottom_meters();
    float top_edge    = area.top_meters();
    if (floor(g_position.y_meters - max_range) < bottom_edge)
    {
        bottom_edge = floor(g_position.y_meters - max_range);
        change = true;
    }
    if (ceil(g_position.y_meters + max_range) > top_edge)
    {
        top_edge = floor(g_position.y_meters + max_range);
        change = true;
    }
    // If bounds were modified, update observed area record.
    if (change)
    {
        observed_area_writer oa_writer = area.writer();
        oa_writer.left_meters   = left_edge;
        oa_writer.right_meters  = right_edge;
        oa_writer.bottom_meters = bottom_edge;
        oa_writer.top_meters    = top_edge;
        oa_writer.update_row();
    }
}    


void create_vertex(world_coordinate_t pos, float heading_degs)
{
    uint32_t obs_num = s_next_vertex_id++;
    gaia_log::app().info("Performing observation {} at {},{} heading {}", 
        obs_num, pos.x_meters, pos.y_meters, heading_degs);
    sensor_data_t data;
    map_coord_t coord = {
        .x_meters = pos.x_meters,
        .y_meters = pos.y_meters,
        .heading_degs = heading_degs
    };
    calculate_range_data(coord, data);

    // Get ego
    ego_t& ego = *(ego_t::list().begin());
    uint32_t graph_id = ego.current_graph_id();
    
    // Get most recent obesrvation.
    vertices_t prev_vert = ego.latest_observation().vertex();
    positions_t prev_pos = prev_vert.position();
    float prev_x_meters, prev_y_meters, prev_heading_degs;
    if (prev_pos)
    {
        prev_x_meters = prev_pos.x_meters();
        prev_y_meters = prev_pos.y_meters();
        prev_heading_degs = prev_pos.heading_degs();
    }
    else
    {
        prev_x_meters = pos.x_meters;
        prev_y_meters = pos.y_meters;
        prev_heading_degs = heading_degs;
    }

    gaia_id_t vertex_id = vertices_t::insert_row(
        obs_num,          // id
        graph_id,         // graph_id
        g_now             // timestamp
    );

    // create position record
    gaia_id_t pos_id = positions_t::insert_row(
        pos.x_meters,     // x_meters
        pos.y_meters,     // y_meters
        heading_degs      // heading_degs
    );

    // create range_data record
    gaia_id_t range_id = range_data_t::insert_row(
        data.num_radials,     // num_radials
        data.bearing_degs,    // bearing_degs
        data.range_meters     // distance_meters
    );

    // create movement record
    float d_degs = heading_degs - prev_heading_degs;
    if (d_degs < 0.0f)
    {
        d_degs += 360.0f;
    }
    gaia_id_t movement_id = movements_t::insert_row(
        g_position.x_meters - prev_x_meters,    // dx_meters
        g_position.y_meters - prev_y_meters,    // dy_meters
        d_degs                                        // dheading_degs
    );

    // create references
    vertices_t v = vertices_t::get(vertex_id);
    v.position().connect(pos_id);
    v.range_data().connect(range_id);
    v.motion().connect(movement_id);

    update_observed_area(ego);

    // create edge
    if (prev_vert)
    {
        edges_t::insert_row(
            graph_id,         // graph_id
            prev_vert.id(),   // src_id
            obs_num           // dest_id
        );
    }

    // update latest_observation
    latest_observation_writer lo_writer = ego.latest_observation().writer();
    lo_writer.vertex_id = obs_num;
    lo_writer.update_row();
}


////////////////////////////////////////////////////////////////////////
// Non-rule API
// The functions here must manage their own transactions.

void seed_database(float x_meters, float y_meters)
{
    // There shouldn't be any transaction conflicts as this is the first
    //  operation on the database so ignore the try/catch block.
    gaia::db::begin_transaction();

    ////////////////////////////////////////////
    // Records
    graphs_t::insert_row(1);  // id
    gaia_id_t ego_id = ego_t::insert_row(1);  // current_graph_id
    ego_t ego = ego_t::get(ego_id);

    gaia_id_t latest_observation_id = latest_observation_t::insert_row(
        0             // observation_id
    );

    // Area map and observed area start out with same dimensions. 
    float left = floorf(x_meters - (c_range_sensor_max_meters+1));
    float bottom = floorf(y_meters - (c_range_sensor_max_meters+1));
    float right = ceilf(x_meters + (c_range_sensor_max_meters+1));
    float top = ceilf(y_meters + (c_range_sensor_max_meters+1));

    gaia_id_t world_id = observed_area_t::insert_row(
        left,     // left_meters
        right,    // right_meters
        top,      // top_meters
        bottom    // bottom_meters
    );

    // Working map record.
    // Create local context to not worry about variable name conflicts.
    gaia_id_t area_id = area_map_t::insert_row(
        0,        // blob_id
        left,     // left_meters
        right,    // right_meters
        top,      // top_meters
        bottom,   // bottom_meters
        0,        // num_rows
        0         // num_cols
    );

    gaia_id_t destination_id = destination_t::insert_row(
        x_meters,     // x_meters
        y_meters,     // y_meters
        0.0           // departure_time_sec
    );

    ////////////////////////////////////////////
    // Relationships
    ego.destination().connect(destination_id);
    ego.world().connect(world_id);
    ego.latest_observation().connect(latest_observation_id);
    ego.map().connect(area_id);

    ////////////////////////////////////////////
    // All done
    gaia::db::commit_transaction();
}

} // namespace slam_sim

