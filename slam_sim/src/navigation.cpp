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

using gaia::slam::graphs_t;
using gaia::slam::edges_t;
using gaia::slam::vertices_t;

using gaia::slam::area_map_t;
using gaia::slam::working_map_t;
//using gaia::slam::error_corrections_t;

using gaia::slam::area_map_writer;
//using gaia::slam::working_map_writer;


// Mock function to calculate error and optimize graph.
void optimize_graph(graphs_t& graph)
{
    gaia_log::app().info("Calculating error (mock)");
    for (vertices_t v: graph.observations())
    {
        // Just iterate through the observations in the graph to show
        //  how it's done.
        gaia_log::app().info("Vertex {} at {},{}", v.id(),
            v.position().pos_x_meters(), v.position().pos_y_meters());
    }
    for (edges_t e: graph.edges())
    {
        // Just iterate through the edges in the graph to show
        //  how it's done.
        gaia_log::app().info("Edge connecting vertices {} and {}",
            e.src().id(), e.dest().id());
    }
    // When the graph is optimized, the position data for each observation
    //  is subject to being updated.
}


//void export_area_map()
//{
//    static int32_t ctr = 0;
//    string fname("map_" + std::to_string(ctr) + ".pgm");
//    gaia_log::app().info("Building map {}", fname);
//    g_area_map->export_as_pnm(fname);
//    ctr++;
//}


// TODO move to rules_api.cpp
void build_area_map(destination_t& dest, area_map_t& am, 
    observed_area_t& bounds)
{
#warning "Need to adapt constructor to accept new bounds"
    // Each time we build an area map 
    occupancy_grid_t area_map(am, true);
    for (graphs_t& g: graphs_t::list())
    {
        for (vertex_t& v: g.vertices())
        {
            gaia_log::app().info("Pulling sensor data from {}:{}", 
                g.id(), v.id());
            area_map->apply_sensor_data(v);
        }
    }
    // Build paths to destination
    area_map.trace_routes(dest);
    // Update blob ID in database
    area_map_writer writer = am.writer();
    writer.blob_id = area_map.blob_id();
    writer.update_row();
//    export_area_map();
}


void build_working_map(destination_t& dest, working_map_t& wm)
{
    // TODO rebuild the working map
    // In the meantime, just 'touch' the record by updating the
    //  change counter.
//    working_map_writer writer = wm.writer();
//    writer.change_counter = wm.change_counter() + 1;
//    writer.update_row();
}

} // namespace slam_sim
