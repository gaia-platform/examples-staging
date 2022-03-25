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
#include "occupancy.hpp"
#include "sensor_data.hpp"

namespace slam_sim
{

// Width (height) of grids in occupancy grid.
constexpr float MAP_NODE_WIDTH_METERS = 0.1;

// Max distance range sensors can get range data.
constexpr double RANGE_SENSOR_MAX_METERS = 4.0;
// Width of range sensor sweep in front of bot.
constexpr double RANGE_SENSOR_SWEEP_DEGS = 90.0;
constexpr int32_t NUM_RANGE_RADIALS = 45;


////////////////////////////////////////////////
// Initialization

void seed_database();


////////////////////////////////////////////////
// Operation

// Determines if it's time to perform a graph optimization.
bool optimization_required();

// Stub function to graph optimization. Contents of function show how
//  to iterate through a graph's nodes ('observations') and edges.
void optimize_graph(gaia::slam::graphs_t&);

// Generates a map (binary occupancy grid) and saves it to disk.
void build_map();
void build_map(gaia::slam_sim::graphs_t, gaia::slam_sim::observerd_area_t&);

} // namespace slam_sim
