#pragma once
#include "gaia_slam.h"
#include "sensor_data.hpp"

namespace slam_sim
{

// Flag to indicate app should exit. Normally this is 0. When it's time
//  to quit it's set to 1.
extern int32_t g_quit;
constexpr int32_t EXIT_AFTER_X_PATHS = 10;

// Flags to indicate state of Alice's movement. This information is
//  stored in the present path.
//      'done' indicates that path has been completed.
//      'starting' indicates that path is just started.
//      'active' indicates that Alice is moving away from a landmark and 
//          toward a destination.
//      'find_landmark' indicaates that exploration is over and Alice is
//          looking for a landmark to get a position fix.
constexpr int32_t PATH_STATE_STARTING = 1;
constexpr int32_t PATH_STATE_ACTIVE = 2;
constexpr int32_t PATH_STATE_FIND_LANDMARK = 4;
constexpr int32_t PATH_STATE_DONE = 8;

// How near to the destination we have to be to say "close enough".
constexpr double DESTINATION_RADIUS_METERS = 0.5;

//
constexpr double INTER_OBSERVATION_DIST_METERS = 0.5;

// How close to a landmark we need to be to be able to use it as a
//  position fix.
constexpr double LANDMARK_DISTANCE_METERS = 1.0;
// How close Alice has to be to see the landmark.
constexpr double LANDMARK_VISIBILITY_METERS = 3.0;

constexpr const char* DEFAULT_WORLD_MAP_PATH = "../data/world_map.json";


////////////////////////////////////////////////
// Going places

// Creates a record in the pending_destination table, or updates
//  the record that is already there.
void request_new_destination(double x_meters, double y_meters);

// Select position to go to. Default beahvior is to explore environment.
void select_destination();

// Sets path.state to FIND_LANDMARK and selects a landmark to go to.
void select_landmark_destination();

// Called in unusual situations, such as if Alice detects a collision or
//  senses that one is imminent. Brings the bot to a halt and performs
//  no further actions. If called when bot is already stopped then 
//  request is ignored.
void full_stop();

// Moves Alice toward one step toward the destination. The working map
//  is consulted to to determine a path to follow. Alice moves forward
//  a fixed straight-line distance and then stops and creates an observation.
void move_toward_destination();

// Start a new path. This creates an initial observation and links it
//  to a path.
void create_new_path();

// Initialize a new path and assign it its first observation.
void init_path(const gaia::slam::observations_t&);


////////////////////////////////////////////////
// Seeing people

// Performs a full sensor sweep of the environment and creates a record
//  in the 'observations' table.
// First brings Alice to a halt if not already stopped.
// Creates observations record, updates estimated_position record.
void create_observation(gaia::slam::paths_t&);

// Do a sensor sweep from at the stated position.
void perform_sensor_sweep(double pos_x_meters, double pos_y_meters, 
    utils::sensor_data_t& data);


////////////////////////////////////////////////
// Mapping

// Given position fixes at the start and end point of the path, estimate
//  DR error and update DR error estimate.
// Updates error_correction record.
void calc_path_error(gaia::slam::paths_t& path);

// Generates a low-res map off the area with path information to destination.
// Stores in 'area_map' record.
void build_area_map();

// Generates a high-res map of the area, based on previously acquired 
//  and calibrated data. 
// Stores output in 'local_map' record.
void build_local_map();

// Generates a high-res map of the area using recently acquired sensor data,
//  aligned as best as possible. Builds a path map to destination, using
//  area map to provide boundary conditions, and obstacle information from
//  local map to supplement recently acquired sensor data.
// Updates working_map record.
void build_working_map();


} // namespace slam_sim

