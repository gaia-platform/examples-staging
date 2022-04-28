////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
#pragma once

#include <stdint.h>

namespace slam_sim
{

////////////////////////////////////////////////////////////////////////
// Supporting types. X,Y values are used in many different contexts. To
//  help avoid mixing paradigms, different types are defined for the
//  different contexts.

// Physical location in world space.
// NOTE: this is in world coordinates (i.e., increasing x,y is up/right).
struct world_coordinate_t
{
    float x_meters;
    float y_meters;
};

// Bounding area of map in world space.
// NOTE: this is in world coordinates (i.e., increasing x,y is up/right).
struct map_size_t
{
    float x_meters;
    float y_meters;
};

// Dimensions of map (occupancy grid).
struct grid_size_t
{
    uint32_t rows;
    uint32_t cols;
};

// Node location in grid.
// NOTE: this is in image coordinates (i.e., 0,0 is top left).
struct grid_coordinate_t
{
    uint32_t x;
    uint32_t y;
};

// Distance from one grid node to another, in units of grid coordinates.
// NOTE: this is in image coordiantes (i.e., increasing x,y is down/right).
struct node_offset_t
{
    int32_t dx;
    int32_t dy;
};

// Storage index of a node w/in the 2D grid.
struct grid_index_t
{
    // The index here is unsigned in order to help detect improper usage 
    //  (e.g., index of -1). 
    uint32_t idx;
};
constexpr uint32_t c_invalid_grid_idx = 0xffffffff;


} // namespace slam_sim

