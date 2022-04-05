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

#include <queue>
#include <vector>

#include "gaia_slam.h"

#include "sensor_data.hpp"

namespace slam_sim
{

// Characteristics of a node in the map, including distance from this node
//  to a/the destination.
// Occupied: how many times the node has been traveled through.
// Observed: how many times the node has been seen.
// Boundary: how many times the node had an obstruction detection in it.
struct map_node_t
{
    // Index of node that's one-closer to destination (-1 for no parent)
    int32_t parent_idx;
    // Direction toward destination. This is moving toward a node that's
    //  X-parents removed (e.g., 5)
    float direction_degs;

    float occupied;
    float observed;
    float boundary;

    // Cost to traverse this node. This is computed as a function of the
    //  node's characteristics.
    float traversal_cost;

    // Total cost to reach destination. This is a unitless measure that
    //  includes biases to avoid heavily trafficed areas to encourage
    //  exploration
    float path_cost;

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


struct node_offset_t
{
    int32_t dx;
    int32_t dy;
};

struct grid_index_t
{
    uint32_t idx;
};



class occupancy_grid_t
{
public:
    occupancy_grid_t(float node_width_meters, world_coordinate_t top_left,
        world_coordinate_t bottom_right);
    occupancy_grid_t(gaia::slam::area_map_t&);
    occupancy_grid_t(gaia::slam::working_map_t&);
    ~occupancy_grid_t();

     / Initialization
    // Resets the map grid.
    void clear();
    // Initializes as inner map using outer map to set boundary conditions.
    // This is meant to be used to build a higher resolution local map to use
    //  based on a larger lower resolution area map.
    //void embed(const occupancy_grid_t& surrounding);

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

    // This will be fixed for a given map resolution.
    float m_node_size_meters;

    // These fields are data dependent. A copy of them is stored in the DB
    //  alongside a pointer to the memory blob storing the grid itself.
    grid_size_t m_grid_size;
    map_size_t m_map_size;
    world_coordinate_t m_bottom_left;
    // Note that positive coordiantes are rightward and upward.

    // Manage array memory directly, so it can be cached in a blob and 
    //  rehydrated as necessary. If ID is <0 then memory is owned by
    //  this object, if >=0 then memory is owned by blob_cache.
    int32_t m_blob_id;
    map_node_t* m_grid;
    map_node_flags_t* m_grid_flags;
    //std::vector<map_node_t> m_grid;
    //std::vector<map_node_flags_t> m_grid_flags;

    ////////////////////////////////////////////
    // Path-finding. This is an algorithm derived from D*, ported from
    //  different project (used w/ permission).
    std::queue<map_node_t*> m_queue;
    void add_node_to_stack(map_node* root_node, grid_index_t root_idx, 
        node_offset_t offset, const float traverse_wt = 1.0f);

    world_coordinate_t m_destination;
};

} // namespace slam_sim

