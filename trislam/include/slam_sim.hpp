////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Main header and API for SLAM simulation.
//
// The SLAM simulator represents a mocked-up SLAM implementation for
//  the sake of showing how a pose graph can be constructed and
//  iterated through, and generating a map (binary occupancy grid) 
//  from the contents of the graph.
//
////////////////////////////////////////////////////////////////////////

#pragma once

#include "gaia_slam.h"

#include "constants.hpp"
#include "coord_list.hpp"
#include "occupancy.hpp"
#include "sensor_data.hpp"

namespace slam_sim
{

////////////////////////////////////////////////
// Rules API
// This is the interface that is expected to invoked by rules
//  (i.e., in the ruleset file). A transaction is expected to
//  already be open when these are called.

// Determines if it's time to perform a graph optimization.
bool optimization_required();

// Stub function to graph optimization. Contents of function show how
//  to iterate through a graph's nodes ('observations') and edges.
void optimize_graph(gaia::slam::graphs_t&);

// Generates a map (binary occupancy grid) and saves it to disk.
void build_map(const gaia::slam::graphs_t&, const gaia::slam::observed_area_t&);


////////////////////////////////////////////////
// Utility

void create_observation(map_coord_t& prev, map_coord_t& coord);

// Similar to 'build_map(...)' above for the Rules API.
// This version manages its own transactions and fetches necessary info
//  from the database.
void build_map();


////////////////////////////////////////////////
// Internal

void seed_database(float start_x_meters, float start_y_meters);

void create_observation(map_coord_t& prev, map_coord_t& coord);
void calculate_range_data(map_coord_t& coord, sensor_data_t& data);

void load_world_map(const char* map_file);

} // namespace slam_sim

