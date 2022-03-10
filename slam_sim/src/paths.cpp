////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <assert.h>
#include <math.h>

#include <iostream>

#include "slam_sim.hpp"

#include "gaia/direct_access/auto_transaction.hpp"

namespace slam_sim
{


// Dev code. Hardcode destinations during development.

constexpr double X0_METERS = 1.0;
constexpr double Y0_METERS = 1.0;

constexpr double X1_METERS = 3.5;
constexpr double Y1_METERS = 6.0;


using gaia::slam::observations_t;
using gaia::slam::ego_t;
using gaia::slam::paths_t;
using gaia::slam::destination_t;
using gaia::slam::estimated_position_t;

using gaia::slam::ego_writer;
using gaia::slam::paths_writer;
using gaia::slam::destination_writer;
using gaia::slam::estimated_position_writer;


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
