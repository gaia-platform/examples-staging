////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

// Globals for this namespace

#include <stdint.h>

#include "occupancy.hpp"

namespace slam_sim
{

int32_t g_quit = 0;
world_coordinate_t g_position = { .x_meters = 0.0, .y_meters = 0.0 };
float g_heading_degs = 0.0f;

std::vector<world_coordinate_t> g_destinations;
uint32_t g_next_destination = 0;

double g_now = 0.0;

bool g_running = false;

} // namespace slam_sim

