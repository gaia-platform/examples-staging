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
#include "landmark_description.hpp"


namespace slam_sim
{

static int32_t s_next_observation_id = 1;

using gaia::common::gaia_id_t;

using utils::sensor_data_t;

////////////////////////////////////////////////////////////////////////
// Rule API
// The functions here are expected to be called from within an active
//  transaction


#error "Rewrite this"
void create_observation(paths_t& path)
{
    // Get position and do a sensor sweep.
    double actual_x_meters = -1.0;
    double actual_y_meters = -1.0;
    for (sim_actual_position_t& sap: sim_actual_position_t::list())
    {
        actual_x_meters = sap.x_meters();
        actual_y_meters = sap.y_meters();
        break;
    }
    double pos_x_meters = -1.0;
    double pos_y_meters = -1.0;
    double dx_meters = -1.0;
    double dy_meters = -1.0;
    for (estimated_position_t& ep: estimated_position_t::list())
    {
        pos_x_meters = ep.x_meters();
        pos_y_meters = ep.y_meters();
        dx_meters = ep.dx_meters();
        dy_meters = ep.dy_meters();
        break;
    }
    double heading_degs = utils::R2D * atan2(pos_x_meters, pos_y_meters);
    double range_meters = sqrt(dx_meters*dx_meters + dy_meters*dy_meters);
    gaia_log::app().info("Performing observation {} at {},{}", 
        s_next_observation_id, pos_x_meters, pos_y_meters);
    sensor_data_t data;
//    double x_offset_meters, y_offset_meters;
//    load_position_offset(x_offset_meters, y_offset_meters);
//    gaia_log::app().info("Position offset at {},{}", 
//        x_offset_meters, y_offset_meters);
printf("SENSOR sweep for obs %d\n", next_observation_id);
    perform_sensor_sweep(actual_x_meters, actual_y_meters, data);

    // Create an observation record, storing sensor data.
    // This is the ID of the new observation. Call it 'num' here so to not
    //  get confused with gaia IDs.
    uint32_t obs_num = next_observation_id++;
    if (path.num_observations() == 0)
    {
        // For first observation, don't store position delta.
        dx_meters = 0.0;
        dy_meters = 0.0;
        heading_degs = 0.0;
        range_meters = 0.0;
    }
    gaia_id_t new_obs_id = observations_t::insert_row(
        obs_num,              // id
        pos_x_meters,         // pos_x_meters
        pos_y_meters,         // pos_y_meters
        actual_x_meters,      // actual_x_meters
        actual_y_meters,      // actual_y_meters
        dx_meters,            // dx_meters
        dy_meters,            // dy_meters
        heading_degs,         // heading_degs
        range_meters,         // dist_meters
        data.num_radials,     // num_radials
        data.range_meters     // distance_meters
    );
    observations_t new_obs = observations_t::get(new_obs_id);

    update_landmarks(path, pos_x_meters, pos_y_meters, obs_num, data);

    // Connect observation to path and to previous observation, if present.
    // Make a copy of the number of observations while we modify path.
    uint32_t number_of_observations = path.num_observations();
    paths_writer p_writer = path.writer();
    if (number_of_observations == 0)
    {
        // First observation.
        p_writer.start_obs_id = obs_num;
        // For first observation, don't store position delta.
        dx_meters = 0.0;
        dy_meters = 0.0;
        heading_degs = 0.0;
        range_meters = 0.0;
    }
    else
    {
        // Build an edge to connect to the previous observation.
        gaia_id_t edge_id = edges_t::insert_row(obs_num);
        // Now link observations to the edge.
        new_obs.reverse_edge().connect(edge_id);
        if (number_of_observations == 1)
        {
            // Second observation so this is the first edge. Get
            //  connection info directly from
            observations_t prev_obs = path.first_observation();
            prev_obs.forward_edge().connect(edge_id);
        }
        else
        {
            // Get connection info from previous edge.
            observations_t prev_obs = path.latest_observation();
            prev_obs.forward_edge().connect(edge_id);
        }

    }
    p_writer.latest_obs_id = obs_num;
    p_writer.num_observations = path.num_observations() + 1;
    p_writer.update_row();
}


////////////////////////////////////////////////////////////////////////
// Non-rule API
// The functions here must manage their own transactions.


void seed_database(double initial_x_meters, double initial_y_meters)
{
    // There shouldn't be any transaction conflicts as this is the first
    //  operation on the database, so ignore the try/catch block.
    gaia::db::begin_transaction();
    //
    gaia_id_t ego_id = ego_t::insert_row(0);  // current_path_id
    ego_t ego = ego_t::get(ego_id);

// TODO FINISH ME

    ////////////////////////////////////////////
    // Position and destination
    gaia_id_t destination_id = destination_t::insert_row(
        0.0,          // x_meters
        0.0,          // y_meters
        0             // expected_arrival
    );
    gaia_id_t position_id = estimated_position_t::insert_row(
        0.0,          // x_meters
        0.0,          // y_meters
        0.0,          // x_meters
        0.0           // y_meters
    );
    gaia_id_t error_correction_id = error_correction_t::insert_row(
        0.0,          // drift_correction
        0.0,          // forward_correction
        0.0           // correction_weight
    );

    ////////////////////////////////////////////
    // Establish relationships
    ego.position().connect(position_id);
    ego.destination().connect(destination_id);
    ego.error().connect(error_correction_id);
    ego.low_res_map().connect(area_map_id);
    ego.high_res_map().connect(local_map_id);
    ego.working_map().connect(working_map_id);

    ////////////////////////////////////////////
    // All done
    gaia::db::commit_transaction();
}

} // namespace slam_sim
