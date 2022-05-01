////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>

#include "constants.hpp"
#include "occupancy.hpp"

namespace slam_sim
{
//class blob_cache_t;
struct world_coordinate_t;

extern occupancy_grid_t g_navigation_map;

// Flag to indicate app should exit. Normally this is 0. When it's time
//  to quit it's set to 1.
extern int32_t g_quit;

//// Blob caches for area maps and working maps.
//extern blob_cache_t g_area_blobs;

// Bot's position. This is known at the 'infrastructure' level (e.g., ROS)
//  which is where position data is fed to the DB from. This is the current
//  (estimated) position of the bot.
extern world_coordinate_t g_position;
extern float g_heading_degs;

extern std::vector<world_coordinate_t> g_destinations;
extern uint32_t g_next_destination;

// Current simulation time.
extern double g_now;

// Flag to indicate motion state (true=moving, false=halted).
extern bool g_running;

} // namespace slam_sim

