#include <assert.h>

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

using gaia::slam::ego_writer;
using gaia::slam::paths_writer;
using gaia::slam::destination_writer;


////////////////////////////////////////////////////////////////////////
// Rule API
// The functions here are expected to be called from within an active
//  transaction

void select_destination()
{
    // If exploring, move 5 meters east.
    // If returning to landmark, move back to start.
    for (ego_t& e: ego_t::list())
    {
        paths_t path = e.current_path();
        if (path.state() == PATH_STATE_ACTIVE)
        {
            for (destination_t& d: destination_t::list())
            {
                destination_writer writer = d.writer();
                writer.x_meters = X1_METERS;
                writer.y_meters = Y1_METERS;
                writer.update_row();
                break;
            }
        } 
        else if (path.state() == PATH_STATE_FIND_LANDMARK)
        {
            for (destination_t& d: destination_t::list())
            {
                destination_writer writer = d.writer();
                writer.x_meters = X0_METERS;
                writer.y_meters = Y0_METERS;
                writer.update_row();
                break;
            }
        }
        else
        {
            // Destination should only be set when active (i.e., outbound
            //  or returning).
            assert(false);
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


////////////////////////////////////////////////////////////////////////
// Non-rule API
// The functions here must manage their own transactions.

void request_new_destination(double x_meters, double y_meters)
{
    // Not implemented yet.
    assert(false);
}

} // namespace slam_sim

