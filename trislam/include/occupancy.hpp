////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
#pragma once

/***********************************************************************

The occupancy grid here is a 2D map that's built form the output from
a SLAM algorithm. It represents the obstacles observed from a bot's
sensors and is used for the bot to navigate from one location to another
using a pathfinding algorithm, a modified dijkstra pathfinding algorithm
that incorporates additional features to better support exploration.

As elsewhere in the code, positive coordinates are to the up and right.

***********************************************************************/
#include <stdint.h>

#include <vector>

#include "gaia_slam.h"

#include "sensor_data.hpp"

namespace slam_sim
{

// Characteristics of a node in the map. Field descriptions:
//  Occupied: how many times the node has been travelled through.
//  Observed: how many times the node has been seen.
//  Boundary: how many times the node had an obstruction detection in it.
struct map_node_t
{
    float occupied;
    float observed;
    float boundary;

    void clear();
};


// Counter/flags for a node's characteristics when evaluating a sensor sweep.
struct map_node_flags_t
{
    uint8_t occupied;
    uint8_t observed;
    uint8_t boundary;

    void clear();
};


struct grid_size_t
{
    uint32_t rows;
    uint32_t cols;
};


struct grid_coord_t
{
    uint32_t x;
    uint32_t y;
};


struct world_coordinate_t
{
    float x_meters;
    float y_meters;
};


struct map_size_t
{
    float x_meters;
    float y_meters;
};


class occupancy_grid_t
{
public:
    // Map center starts at 0,0 so map bounds are at +/- width/2 and 
    //  +/- height/2.
    occupancy_grid_t(float node_width_meters, world_coordinate_t top_left,
        world_coordinate_t bottom_right);
    ~occupancy_grid_t();

    // Resets the map grid.
    void clear();

    // Returns a reference to tne map node at the specified location.
    map_node_t& get_node(float x_meters, float y_meters);
    map_node_flags_t& get_node_flags(float x_meters, float y_meters);

    // Apply sensor data to map from observation.
    void apply_sensor_data(const gaia::slam::observations_t&);

    void export_as_pnm(std::string file_name);

protected:
    uint32_t get_node_index(float pos_x_meters, float pos_y_meters);
    void apply_radial(float radial_degs, float range_meters, 
        float pos_x_meters, float pos_y_meters);
    void apply_flags();

    float m_node_size_meters;
    grid_size_t m_grid_size;

    map_size_t m_map_size;
    // Positive coordiantes are rightward and upward.
    world_coordinate_t m_bottom_left;

    std::vector<map_node_t> m_grid;
    std::vector<map_node_flags_t> m_grid_flags;

    world_coordinate_t m_destination;
};

} // namespace slam_sim

