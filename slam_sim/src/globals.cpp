////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <cstdint>

#include "occupancy.hpp"

namespace slam_sim
{

int32_t g_quit = 0;

// Area map is not presently protectected against being accessed from
//  multiple threads simultaneously. Simultaneous access shouldn't occur
//  due to the way rules are constructed but protection should still be
//  added. TODO
occupancy_grid_t* g_area_map;

}
