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
// The SLAM simulator represents a simplified SLAM implementation to
//  provide an example of doing SLAM in Gaia. Sensor observations are
//  stored in the database and records are linked to form graphs.
//  Graph traversal is done by just following these links.
//
// The simulated environment is 2D. The bot ("Alice") has range sensors
//  that scan 360 degrees and a camera to identify landmarks.
//
////////////////////////////////////////////////////////////////////////

#pragma once

#include "gaia_slam.h"

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
//  to iterate through a graph's vertices and edges.
void optimize_graph(gaia::slam::graphs_t&);


// Checks to see if known world has extended beyong map boundaries so much
//  that it's time to rebuild the area map.
bool need_to_extend_map(gaia::slam::positions_t& pos, 
    gaia::slam::observed_area_t& bounds);

// Generates a low-res map off the known world and generates coarse path
//  info to the destination.
// Stores output 'area_map' record.
void build_area_map(gaia::slam::destination_t&, gaia::slam::area_map_t&, 
    gaia::slam::observed_area_t&);

// Checks to see if it's time to select a new destination. If so, the
//  destination record is updated and the function returns 'true'. Otherwise
//  returns 'false'.
bool reassess_destination();

// Updates the observed_area record to make sure it includes all observed
//  areas plus the destination.
void update_world_area(gaia::slam::ego_t& ego);

// Instructs hardware layer to move toward destination.
void move_toward_destination();

// Instructs bot to stop. An observation will be taken here.
void full_stop();


////////////////////////////////////////////////
// Support API

// Creates a record in the vertices table using the supplied sensor
//  data
void create_vertex(world_coordinate_t pos, float heading_degs);

// Do a sensor sweep from at the stated position. This would normally be
//  pushed up from the sensor/hardware layer, but polling for it makes
//  the simulation more straightforward.
void generate_sensor_data(float pos_x_meters, float pos_y_meters,
    sensor_data_t& data);

// Generates and exports a map to disk. Filename will be based on ego's
//  timestamp.
void export_map_to_file();

// External request to move to a specific location.
void request_destination(float x_meters, float y_meters);


////////////////////////////////////////////////
// Environmental analysis

// Calculates sensor view from a position in the environment.
void calculate_range_data(map_coord_t& coord, sensor_data_t& data);

////////////////////////////////////////////////
// Initialization

// Params are Alice's starting point.
void seed_database(float initial_x_meters, float iniital_y_meters);

// Loads a .json file that describes the environment.
void load_world_map(const char*);


} // namespace slam_sim
