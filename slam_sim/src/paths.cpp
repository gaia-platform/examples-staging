////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
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

extern int32_t g_quit;

static int32_t next_observation_id = 1;


using gaia::common::gaia_id_t;

using gaia::slam::observations_t;
using gaia::slam::ego_t;
using gaia::slam::paths_t;
using gaia::slam::destination_t;
using gaia::slam::estimated_position_t;
using gaia::slam::edges_t;
using gaia::slam::error_correction_t;
using gaia::slam::area_map_t;
using gaia::slam::local_map_t;
using gaia::slam::working_map_t;
using gaia::slam::landmark_sightings_t;
using gaia::slam::landmarks_t;;
using gaia::slam::sim_position_offset_t;
using gaia::slam::sim_actual_position_t;

using gaia::slam::ego_writer;
using gaia::slam::paths_writer;
using gaia::slam::destination_writer;
using gaia::slam::estimated_position_writer;
using gaia::slam::edges_writer;
using gaia::slam::landmarks_writer;
using gaia::slam::sim_position_offset_writer;
using gaia::slam::sim_actual_position_writer;

using utils::sensor_data_t;
using utils::landmark_description_t;

////////////////////////////////////////////////////////////////////////
// Rule API
// The functions here are expected to be called from within an active
//  transaction

// Dev code. Hardcode destinations during development.
constexpr double X_DEST_OFFSET_METERS = 8.0;
constexpr double Y_DEST_OFFSET_METERS = -8.8;


void select_destination()
{
    ego_t e = *(ego_t::list().begin());
    // Sanity check
    paths_t path = e.current_path();
    assert(path.state() == PATH_STATE_ACTIVE);
    // Estimate how long it should take to get to the destination.
    estimated_position_t pos = e.position();
    double dx_meters = X_DEST_OFFSET_METERS;
    double dy_meters = Y_DEST_OFFSET_METERS;
    double dist_meters = sqrt(dx_meters*dx_meters + dy_meters*dy_meters);
    if (dist_meters < INTER_OBSERVATION_DIST_METERS)
    {
        dist_meters = INTER_OBSERVATION_DIST_METERS;
    }
    double hops = ceil(dist_meters / INTER_OBSERVATION_DIST_METERS);
    // Pad the arrival time in case we have to take a non-direct route.
    hops *= 1.5;
    // Set destination coordinates.
    destination_t d = *(destination_t::list().begin());
    destination_writer writer = d.writer();
    writer.x_meters = pos.x_meters() + X_DEST_OFFSET_METERS;
    writer.y_meters = pos.y_meters() + Y_DEST_OFFSET_METERS;
    writer.expected_arrival = (int32_t) ceil(hops);
    writer.update_row();
    gaia_log::app().info("Setting destination to {},{}",
        d.x_meters(), d.y_meters());
}


void select_landmark_destination()
{
    // Return to starting point
    ego_t e = *(ego_t::list().begin());
    // Sanity check
    paths_t path = e.current_path();
    assert((path.state() & PATH_STATE_STARTING) == 0);
    assert((path.state() & PATH_STATE_DONE) == 0);
    // Set 'find-landmark' state if it's not already set.
    if (path.state() == PATH_STATE_ACTIVE)
    {
        paths_writer writer = path.writer();
        writer.state = PATH_STATE_FIND_LANDMARK;
        writer.update_row();
    }
    // Set destination coordinates.
    sim_position_offset_t& spo = *(sim_position_offset_t::list().begin());
    destination_t d = *(destination_t::list().begin());
    destination_writer writer = d.writer();
    writer.x_meters = spo.dx_meters();
    writer.y_meters = spo.dy_meters();
    writer.update_row();
    gaia_log::app().info("Setting return destination to {},{}",
        d.x_meters(), d.y_meters());
}


// Initialize a path with the first observation.
void init_path(const observations_t& o)
{
    for (ego_t& e: ego_t::list())
    {
        paths_t path = e.current_path();
        paths_writer writer = path.writer();
        writer.start_obs_id = o.id();
        writer.latest_obs_id = o.id();
        writer.num_observations = 1;
        writer.state = slam_sim::PATH_STATE_ACTIVE;
        writer.update_row();

        // There's only one ego, but it doesn't hurt to break out anyway.
        break;
    }
}


void create_new_path()
{
    // Update ego with next path ID and get that ID to privide it to
    //  the new path.
    uint32_t next_path_id = 0;
    for (ego_t& e: ego_t::list())
    {
        next_path_id = e.current_path_id() + 1;
        ego_writer writer = e.writer();
        writer.current_path_id = next_path_id;
        writer.update_row();
        break;
    }

    // Create path.
    paths_writer writer = paths_writer();
    writer.id = next_path_id;
    writer.state = PATH_STATE_STARTING;
    writer.insert_row();

    // See if it's time to quit.
    if (next_path_id >= EXIT_AFTER_X_PATHS)
    {
        g_quit = 1;
    }
}


void full_stop()
{
    // Unimplemented.
    // When a stop is requested we need to stop the wheels and then
    //  update our position. The simulated robot doesn't actually update
    //  its position in a smooth manner presently, instead jumping ahead
    //  to an interim destination. Once motion tracking is done smoothly
    //  over time then fill out this function.
    assert(false);
}


// Update estimated position. This will trigger creation of a new
//  observation.
// In a real robot, this would engage the wheels and drive a specified
//  distance, in turn updating the estimated position. Here we just jump
//  ahead to estimating the position.
void move_toward_destination()
{
    // TODO Consult map and find new checkpoint to move to.
    // For now, move in the direction of the present destination.
    // Update actual movement and perceived movement.
    // For now these are one and the same .
    // TODO Introduce error

    double dest_x_meters, dest_y_meters;
    for (destination_t& d: destination_t::list())
    {
        dest_x_meters = d.x_meters();
        dest_y_meters = d.y_meters();
        break;
    }
    double pos_x_meters, pos_y_meters;
    estimated_position_writer writer;
    for (estimated_position_t& ep: estimated_position_t::list())
    {
        pos_x_meters = ep.x_meters();
        pos_y_meters = ep.y_meters();
        writer = ep.writer();
        break;
    }
    double dx = dest_x_meters - pos_x_meters;
    double dy = dest_y_meters - pos_y_meters;
    double dist = sqrt(dx*dx + dy*dy);
    // Number of steps to get to destination.
    double hops = dist / INTER_OBSERVATION_DIST_METERS;
    if (hops < 1.0)
    {
        hops = 1.0;
    }
    dx /= hops;
    dy /= hops;
    writer.x_meters = pos_x_meters + dx;
    writer.y_meters = pos_y_meters + dy;
    writer.dx_meters = dx;
    writer.dy_meters = dy;
    writer.update_row();
gaia_log::app().info("Moving from {},{} by {},{}", pos_x_meters, pos_y_meters, dx, dy);

    for (sim_actual_position_t& sap: sim_actual_position_t::list())
    {
        sim_actual_position_writer sapw = sap.writer();
        sapw.x_meters = sap.x_meters() + dx;
        sapw.y_meters = sap.y_meters() + dy;
        sapw.update_row();
    }
}


//// There are 2 positional reference frames for Alice, that of the world
////  and that relative to Alice's starting point. The map is in world
////  coordinates while Alice doesn't know the world coordinates and so
////  uses the starting position as 0,0. This function returns the offset
////  from Alice's coordinate frame to the world coordinate frame.
//static void load_position_offset(double& dx_meters, double& dy_meters)
//{
//    // The positional offset doesn't change. Keep a cache of the offset
//    //  to avoid having to look it up each time.
//    // Initial position must be positive as world map doesn't have
//    //  any negative coords.
//    static double s_dx_meters = -1.0;
//    static double s_dy_meters = -1.0;
//    // 
//    if (s_dx_meters < 0.0)
//    {
//        for (sim_position_offset_t& spo: sim_position_offset_t::list())
//        {
//            gaia_log::app().info("Reading position offset");
//            s_dx_meters = spo.dx_meters();
//            s_dy_meters = spo.dy_meters();
//            break;
//        }
//    }
//    dx_meters = s_dx_meters;
//    dy_meters = s_dy_meters;
//}


// Helper function when creating an observation, to store landmark
//  sightings and create new landmark records if necessary.
static void update_landmarks(paths_t& path, double pos_x_meters,
    double pos_y_meters, uint32_t obs_num, sensor_data_t& data)
{
    // Create landmark sighted records.
    // Keep track of nearest one.
    int32_t nearest_id = -1;
    double nearest_meters = -1.0;
    for (landmark_description_t& ld: data.landmarks_visible)
    {
        double dx = ld.x_meters - pos_x_meters;
        double dy = ld.y_meters - pos_y_meters;
        double range = sqrt(dx*dx + dy*dy);
        double bearing = utils::R2D * atan2(dx, dy);
        if (bearing < 0.0)
        {
            bearing += 360.0;
        }
        if ((nearest_meters < 0.0) || (range < nearest_meters))
        {
            nearest_id = ld.id;
            nearest_meters = range;
        }
        landmark_sightings_t::insert_row(
            range,            // range_meters
            bearing,          // bearing_degs
            obs_num,          // observaation_id
            ld.id             // landmark_id
        );
    }
    if ((path.num_observations() == 0) && (nearest_id < 0))
    {
        std::cerr << "Start position must be near visible landmark.\n";
        exit(1);
    }
    // Create landmark records of sighted landmarks for those that don't
    //  exist.
    for (landmark_description_t& ld: data.landmarks_visible)
    {
        assert(nearest_id >= 0);
        landmarks_t mark = *(landmarks_t::list()
            .where(gaia::slam::landmarks_expr::id == (uint32_t) ld.id).begin());
        if (!mark)
        {
gaia_log::app().info("Creating landmark {} at {},{}", ld.name, ld.x_meters, ld.y_meters);
            // Landmark record doesn't exist. Create it.
            float confidence = 0.1;
            if ((path.num_observations() == 0) && (ld.id == nearest_id))
            {
                // This is the nearest landmark on the first observation.
                // Use this as the anchor point.
                confidence = 1.0;
                assert(pos_x_meters == 0.0);
                assert(pos_y_meters == 0.0);
            }
            landmarks_t::insert_row(
                ld.id,            // landmark_id
                ld.name.c_str(),  // description
                ld.x_meters,      // x_meters
                ld.y_meters,      // y_meters
                confidence        // confidence
            );
        }
        else if (mark.confidence() < 1.0)
        {
            // Update landmark position and confidence if sighting
            //  is closer to landmark than previous sighting(s).
            double dx = ld.x_meters - pos_x_meters;
            double dy = ld.y_meters - pos_y_meters;
            double range = sqrt(dx*dx + dy*dy);
            double landmark_dx = ld.x_meters - pos_x_meters;
            double landmark_dy = ld.y_meters - pos_y_meters;
            double landmark_range = 
                sqrt(landmark_dx*landmark_dx + landmark_dy*landmark_dy);
            if (range < landmark_range)
            {
gaia_log::app().info("Updating landmark {} to {},{}", ld.name, ld.x_meters, ld.y_meters);
                // Update record.
                landmarks_writer writer = mark.writer();
                writer.x_meters = dx;
                writer.y_meters = dy;
                writer.update_row();
            }
        }
    }
}


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
        next_observation_id, pos_x_meters, pos_y_meters);
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

void request_new_destination(double x_meters, double y_meters)
{
    (void) x_meters;
    (void) y_meters;
    // Not implemented yet.
    assert(false);
}


void seed_database(double initial_x_meters, double initial_y_meters)
{
    // There shouldn't be any transaction conflicts as this is the first
    //  operation on the database, so ignore the try/catch block.
    gaia::db::begin_transaction();
    //
    gaia_id_t ego_id = ego_t::insert_row(0);  // current_path_id
    ego_t ego = ego_t::get(ego_id);

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
    // Maps
    gaia_id_t area_map_id = area_map_t::insert_row(
        0.0,      // left_meters
        0.0,      // right_meters
        0.0,      // top_meters
        0.0,      // bottom_meters
        0         // change_counter
    );
    gaia_id_t local_map_id = local_map_t::insert_row(
        0.0,      // left_meters
        0.0,      // right_meters
        0.0,      // top_meters
        0.0,      // bottom_meters
        0         // change_counter
    );
    gaia_id_t working_map_id = working_map_t::insert_row(
        0         // change_counter
    );

    ////////////////////////////////////////////
    // Simulation interface
    sim_position_offset_t::insert_row(
        initial_x_meters,   // dx_meters
        initial_y_meters    // dy_meters
    );

    sim_actual_position_t::insert_row(
        initial_x_meters,   // x_meters
        initial_y_meters    // y_meters
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
