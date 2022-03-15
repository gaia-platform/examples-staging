////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
// 
// Primary API for rules relating to navigation and error estimation.
//
// Maps are meant to provide a means for Alice to determine areas to
//  explore, for navigating between locations, and for avoiding
//  obstacles.
//
// Maps are generated from the output of the SLAM algorithm, using 
//  observation data corrected for estimated errors.
//
////////////////////////////////////////////////////////////////////////

#include <gaia/logger.hpp>

#include "slam_sim.hpp"

namespace slam_sim
{

using gaia::slam::paths_t;
using gaia::slam::observations_t;
using gaia::slam::edges_t;
using gaia::slam::area_map_t;
using gaia::slam::local_map_t;
using gaia::slam::working_map_t;
using gaia::slam::error_correction_t;

using gaia::slam::area_map_writer;
using gaia::slam::local_map_writer;
using gaia::slam::working_map_writer;
using gaia::slam::error_correction_writer;


void calc_path_error(paths_t& path)
{
gaia_log::app().info("Calculating error");
    observations_t head = path.first_observation();
gaia_log::app().info("Error calc start at {},{}", head.pos_x_meters(), head.pos_y_meters());
    edges_t e = head.forward_edge();
    observations_t next = e.next();
    while (next)
    {
        // TODO do something to estimate error
        // For now, just iterate through the observations in the path.
        gaia_log::app().info("Error obs {} at {},{}", next.id(), 
            next.pos_x_meters(), next.pos_y_meters());

        if (!next.forward_edge()) {
            break;
        }
        next = next.forward_edge().next();
    }

    // Updated error correction table.
    for (error_correction_t& ec: error_correction_t::list())
    {
        error_correction_writer writer = ec.writer();
        writer.correction_weight = ec.correction_weight() + 1.0;
        writer.update_row();
        break;
    }
}

void build_area_map(area_map_t& am)
{
    // TODO rebuild the area map
    // In the meantime, just 'touch' the record by updating the
    //  change counter.
    area_map_writer writer = am.writer();
    writer.change_counter = am.change_counter() + 1;
    writer.update_row();
}

void build_local_map(local_map_t& lm)
{
    // TODO rebuild the local map
    // In the meantime, just 'touch' the record by updating the
    //  change counter.
    local_map_writer writer = lm.writer();
    writer.change_counter = lm.change_counter() + 1;
    writer.update_row();
}

void build_working_map(working_map_t& wm)
{
    // TODO rebuild the working map
    // In the meantime, just 'touch' the record by updating the
    //  change counter.
    working_map_writer writer = wm.writer();
    writer.change_counter = wm.change_counter() + 1;
    writer.update_row();
}

} // namespace slam_sim
