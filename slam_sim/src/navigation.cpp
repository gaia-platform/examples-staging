////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

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

using std::string;

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


// Mock function to calculate error and optimize graph.
void optimize_graph(graphs_t& graph)
{
    gaia_log::app().info("Calculating error (mock)");
    for (observations_t o: graph.observations())
    {
        // Just iterate through the observations in the graph to show
        //  how it's done.
        gaia_log::app().info("Observaation {} at {},{}", o.id(),
            o.position().pos_x_meters(), o.position().pos_y_meters());
    }
    for (edges_t e: graph.edges())
    {
        // Just iterate through the edges in the graph to show
        //  how it's done.
        gaia_log::app().info("Edge connecting observations {} and {}",
            e.src().id(), e.dest().id());
    }
    // When the graph is optimized, the position data for each observation
    //  is expected to be updated.
}


void export_area_map()
{
    static int32_t ctr = 0;
    string fname("map_" + std::to_string(ctr) + ".pgm");
    gaia_log::app().info("Building map {}", fname);
    g_area_map->export_as_pnm(fname);
    ctr++;
}


void build_area_map(area_map_t& am)
{
    g_area_map->clear();
    for (paths_t& p: paths_t::list())
    {
        observations_t obs = p.first_observation();
        while (obs)
        {
            // TODO do something with observation data.
            gaia_log::app().info("Pulling sensor data from {}:{}", 
                p.id(), obs.id());
            g_area_map->apply_sensor_data(obs);
            if (!obs.forward_edge()) {
                break;
            }
            obs = obs.forward_edge().next();
        }
    }

    // TODO rebuild the area map
    // In the meantime, just 'touch' the record by updating the
    //  change counter.
    area_map_writer writer = am.writer();
    writer.change_counter = am.change_counter() + 1;
    writer.update_row();

    export_area_map();
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
