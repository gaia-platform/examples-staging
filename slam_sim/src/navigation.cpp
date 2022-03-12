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

#include "slam_sim.hpp"

namespace slam_sim
{

using gaia::slam::paths_t;
using gaia::slam::observations_t;
using gaia::slam::area_map_t;
using gaia::slam::local_map_t;
using gaia::slam::working_map_t;

using gaia::slam::area_map_writer;
using gaia::slam::local_map_writer;
using gaia::slam::working_map_writer;


void calc_path_error(paths_t& path)
{
    printf("Path %d complete\n", path.id());
    observations_t head = path.first_observation();
    printf("Start at %.1f,%.1f\n", head.pos_x_meters(), head.pos_y_meters());
    observations_t next = head.forward_edge().next();
    while (next)
    {
        // TODO do something to estimate error
        // For now, just iterate through the observations in the path.

        printf("  Move %.2f,%.2f to %.1f,%.1f\n", 
            next.dx_meters(), next.dy_meters(),
            next.pos_x_meters(), next.pos_y_meters());
        next = next.forward_edge().next();
    }
}

void build_area_map()
{
    for (area_map_t& m: area_map_t::list())
    {
        // TODO rebuild the area map
        // In the meantime, just 'touch' the record by updating the
        //  change counter.
        area_map_writer writer = m.writer();
        writer.change_counter = m.change_counter() + 1;
        writer.update_row();

        // This isn't necessary as there's only one record, but it does
        //  help keep the code more clear.
        break;  
    }
}

void build_local_map()
{
    for (local_map_t& m: local_map_t::list())
    {
        // TODO rebuild the local map
        // In the meantime, just 'touch' the record by updating the
        //  change counter.
        local_map_writer writer = m.writer();
        writer.change_counter = m.change_counter() + 1;
        writer.update_row();

        // This isn't necessary as there's only one record, but it does
        //  help keep the code more clear.
        break;  
    }
}

void build_working_map()
{
    for (working_map_t& m: working_map_t::list())
    {
        // TODO rebuild the working map
        // In the meantime, just 'touch' the record by updating the
        //  change counter.
        working_map_writer writer = m.writer();
        writer.change_counter = m.change_counter() + 1;
        writer.update_row();

        // This isn't necessary as there's only one record, but it does
        //  help keep the code more clear.
        break;  
    }
}

} // namespace slam_sim
