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

#include "coord_list.hpp"
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
// Rules API

// Determines if it's time to perform a graph optimization.
bool optimization_required();

// Stub function to graph optimization. Contents of function show how
//  to iterate through a graph's nodes ('observations') and edges.
void optimize_graph(gaia::slam::graphs_t&);

// Generates a map (binary occupancy grid) and saves it to disk.
// Version with no parameters manages its own transaction. The version
//  with record references is meant to be called from w/in an existing
//  transaction.
void build_map();
void build_map(const gaia::slam::graphs_t&, const gaia::slam::observed_area_t&);

////////////////////////////////////////////////
// Utility

void create_observation(map_coord_t& prev, map_coord_t& coord);


////////////////////////////////////////////////
// Internal

void seed_database(float start_x_meters, float start_y_meters);

void create_observation(map_coord_t& prev, map_coord_t& coord);
void calculate_range_data(map_coord_t& coord, utils::sensor_data_t& data);

void load_world_map(const char* map_file);

} // namespace slam_sim

