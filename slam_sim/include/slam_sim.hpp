#pragma once
#include "gaia_slam.h"

namespace slam_sim
{

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


////////////////////////////////////////////////
// Seeing people

// Performs a full sensor sweep of the environment and creates a record
//  in the 'observations' table.
// First brings Alice to a halt if not already stopped.
void create_observation();


////////////////////////////////////////////////
// Mapping

// Given position fixes at the start and end point of the path, estimate
//  DR error and update DR error estimate.
// Updates error_correction record.
void calc_path_error();

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

