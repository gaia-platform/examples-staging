////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Primary API for rules relating to path generation.
//
////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <math.h>

#include <iostream>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>

#include "slam_sim.hpp"
#include "line_segment.hpp"


namespace slam_sim
{

static int32_t s_next_observation_id = 1;

using gaia::common::gaia_id_t;

using gaia::slam::edges_t;
using gaia::slam::ego_t;
using gaia::slam::graphs_t;
using gaia::slam::latest_observation_t;
using gaia::slam::movements_t;
using gaia::slam::observations_t;
using gaia::slam::observed_area_t;
using gaia::slam::positions_t;
using gaia::slam::range_data_t;

using gaia::slam::latest_observation_writer;
using gaia::slam::observed_area_writer;

using utils::sensor_data_t;

////////////////////////////////////////////////////////////////////////
// Rule API
// The functions here are expected to be called from within an active
//  transaction


static void update_observed_area(ego_t& ego, map_coord_t coord)
{
    const observed_area_t& area = ego.world();
    bool change = false;
    float left_edge   = area.left_meters();
    float right_edge  = area.right_meters();
    float bottom_edge = area.bottom_meters();
    float top_edge    = area.top_meters();
    if (floor(coord.x_meters - RANGE_SENSOR_MAX_METERS) < left_edge)
    {
        left_edge = floor(coord.x_meters - RANGE_SENSOR_MAX_METERS);
        change = true;
    }
    if (change)
    {
        observed_area_writer oa_writer = observed_area_writer();
        oa_writer.left_meters   = left_edge;
        oa_writer.right_meters  = right_edge;
        oa_writer.bottom_meters = bottom_edge;
        oa_writer.top_meters    = top_edge;
        oa_writer.update_row();
    }
}    

void create_observation(map_coord_t& prev, map_coord_t& coord)
{
    uint32_t obs_num = s_next_observation_id++;
    gaia_log::app().info("Performing observation {} at {},{} heading {}", 
        obs_num, coord.x_meters, coord.y_meters, coord.heading_degs);
    sensor_data_t data;
printf("SENSOR sweep for obs %d\n", obs_num);
    calculate_range_data(coord, data);

    // Get ego
    ego_t& ego = *(ego_t::list().begin());
    uint32_t graph_id = ego.current_graph_id();

    // create position record
    gaia_id_t pos_id = positions_t::insert_row(
        coord.x_meters,       // x_meters
        coord.y_meters,       // y_meters
        coord.heading_degs    // heading_degs
    );
    // create range_data record
    gaia_id_t range_id = range_data_t::insert_row(
        data.num_radials,     // num_radials
        data.bearing_degs,    // bearing_degs
        data.range_meters     // distance_meters
    );
    // create movement record
    gaia_id_t movement_id = movements_t::insert_row(
        coord.x_meters - prev.x_meters,         // dx_meters
        coord.y_meters - prev.y_meters,         // dy_meters
        coord.heading_degs - prev.heading_degs  // dheading_degs
    );
    // create observation record
    gaia_id_t observation_id = observations_t::insert_row(
        obs_num,              // id
        graph_id              // graph_id
    );

    // create references
    observations_t obs = observations_t::get(observation_id);
    obs.position().connect(pos_id);
    obs.range_data().connect(range_id);
    obs.motion().connect(movement_id);

    update_observed_area(ego, coord);

    // create edge
    const latest_observation_t lo = ego.latest_observation();
    const observations_t prev_obs = lo.observation();
    if (prev_obs)
    {
        edges_t::insert_row(
            graph_id,         // graph_id
            prev_obs.id(),    // src_id
            obs_num           // dest_id
        );
    }

    // update latest_observation
    latest_observation_writer lo_writer = latest_observation_writer();
    lo_writer.observation_id = obs_num;
    lo_writer.update_row();
}


////////////////////////////////////////////////////////////////////////
// Non-rule API
// The functions here must manage their own transactions.

void seed_database()
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
    gaia_id_t area_id = observed_area_t::insert_row(
        floor(-RANGE_SENSOR_MAX_METERS),      // left_meters
        ceil(RANGE_SENSOR_MAX_METERS),        // right_meters
        ceil(RANGE_SENSOR_MAX_METERS),        // top_meters
        floor(-RANGE_SENSOR_MAX_METERS)       // bottom_meters
    );

    ////////////////////////////////////////////
    // Relationships
    ego.world().connect(area_id);
    ego.latest_observation().connect(latest_observation_id);

    ////////////////////////////////////////////
    // All done
    gaia::db::commit_transaction();
}

} // namespace slam_sim

