#include <assert.h>
#include <math.h>

#include <iostream>

#include "slam_sim.hpp"
#include "line_segment.hpp"

#include "gaia/direct_access/auto_transaction.hpp"

namespace slam_sim
{


// Dev code. Hardcode destinations during development.

constexpr double X0_METERS = 4.0;
constexpr double Y0_METERS = 0.6;

constexpr double X1_METERS = 5.5;
constexpr double Y1_METERS = 7.5;

static int32_t next_observation_id = 1;


using gaia::common::gaia_id_t;

using gaia::slam::observations_t;
using gaia::slam::ego_t;
using gaia::slam::paths_t;
using gaia::slam::destination_t;
using gaia::slam::estimated_position_t;
using gaia::slam::edges_t;

using gaia::slam::ego_writer;
using gaia::slam::paths_writer;
using gaia::slam::destination_writer;
using gaia::slam::estimated_position_writer;
using gaia::slam::edges_writer;

using utils::sensor_data_t;

////////////////////////////////////////////////////////////////////////
// Rule API
// The functions here are expected to be called from within an active
//  transaction

void select_destination()
{
    // Move to predesignated position.
    for (ego_t& e: ego_t::list())
    {
        paths_t path = e.current_path();
        assert(path.state() == PATH_STATE_ACTIVE);
        for (destination_t& d: destination_t::list())
        {
            destination_writer writer = d.writer();
            writer.x_meters = X1_METERS;
            writer.y_meters = Y1_METERS;
            writer.update_row();
            break;
        }
        break;
    }
}


void select_landmark_destination()
{
    // Move to (return to) predesignated position.
    for (ego_t& e: ego_t::list())
    {
        paths_t path = e.current_path();
        assert((path.state() & PATH_STATE_STARTING) == 0);
        assert((path.state() & PATH_STATE_DONE) == 0);

        // Set find-landmark state if it's not already set.
        if (path.state() == PATH_STATE_ACTIVE)
        {
            paths_writer writer = path.writer();
            writer.state = PATH_STATE_FIND_LANDMARK;
            writer.update_row();
        }

        for (destination_t& d: destination_t::list())
        {
            destination_writer writer = d.writer();
            writer.x_meters = X0_METERS;
            writer.y_meters = Y0_METERS;
            writer.update_row();
            break;
        }
        break;
    }
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
}


void create_observation(paths_t& path)
{
    // Get position and do a sensor sweep.
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
    sensor_data_t data;
    perform_sensor_sweep(pos_x_meters, pos_y_meters, data);

    // Create an observation record, storing sensor data.
    // This is the ID of the new observation. Call it 'num' here so to not
    //  get confused with gaia IDs.
    int32_t obs_num = next_observation_id++;
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
        dx_meters,            // dx_meters
        dy_meters,            // dy_meters
        heading_degs,         // heading_degs
        range_meters,         // dist_meters
        data.num_radials,     // num_radials
        data.range_meters     // distance_meters
    );


    observations_t prev_obs = path.latest_observation().reverse_edge().prev();

    observations_t new_obs;
    gaia_id_t prev_obs_id;

    // Connect observation to path and to previous observation, if present.
    paths_writer writer = path.writer();
    if (path.num_observations() == 0)
    {
        // First observation.
        writer.start_obs_id = obs_num;
        writer.latest_obs_id = obs_num;
        // For first observation, don't store position delta.
        dx_meters = 0.0;
        dy_meters = 0.0;
        heading_degs = 0.0;
        range_meters = 0.0;
    }
    else
    {
        writer.latest_obs_id = obs_num;
        // Build an edge to connect to the previous observation.
        gaia_id_t edge_id = edges_t::insert_row(obs_num);
        // Now link the records.
        new_obs.reverse_edge().connect(edge_id);
        prev_obs.forward_edge().connect(edge_id);
    }
    writer.num_observations = path.num_observations() + 1;
    writer.update_row();
}


////////////////////////////////////////////////////////////////////////
// Non-rule API
// The functions here must manage their own transactions.

void request_new_destination(double x_meters, double y_meters)
{
    // Not implemented yet.
    assert(false);
}

} // namespace slam_sim

