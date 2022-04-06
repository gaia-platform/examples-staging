////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
#pragma once

/***********************************************************************

The occupancy grid here is a 2D map that's built from the output from
a SLAM algorithm. It represents the obstacles observed from a bot's
sensors and is used for the bot to navigate from one location to another
using a pathfinding algorithm, a modified dijkstra pathfinding algorithm
that incorporates additional features to better support exploration.

Two different coordinate schemes are used here. Real world coordinates
are increasing to the up and right, as elsewhere in the code. The map
itself is stored in an array which uses the standard image coordinate
system, with the first element in the array being at the top-left.
The code has to manage the switch between the two.
// TODO FIXME verify that this is indeed the case, and that the switch
//  is being properly handled.

***********************************************************************/
#include <stdint.h>

#include <queue>
#include <vector>

#include "gaia_slam.h"

#include "sensor_data.hpp"

namespace slam_sim
{

////////////////////////////////////////////////////////////////////////
// Supporting types. X,Y values are used in many different contexts. To
//  help avoid mixing paradigms, different types are defined for the
//  different contexts.

// Physical location in world space.
struct world_coordinate_t
{
    float x_meters;
    float y_meters;
};

// Bounding area of map in world space.
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
struct grid_coordinate_t
{
    uint32_t x;
    uint32_t y;
};

// Distance from one grid node to another, in units of grid coordinates.
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


////////////////////////////////////////////////////////////////////////
// The map is composed of several nodes arranged on a grid. 

// Counter/flags for a node's characteristics when evaluating a sensor sweep.
// Keep these flags in a seperate structure to help prevent confusion w/ 
//  similarly named counters in map_node_t.
//    Occupied: if the node has been traveled through.
//    Observed: if the node has been seen.
//    Boundary: if the node had an obstruction detection in it.
struct map_node_flags_t
{
    uint8_t occupied;
    uint8_t observed;
    uint8_t boundary;
    uint8_t state;

    void clear();
};

#define PATH_NODE_FLAG_PROCESSED    0x01
#define PATH_NODE_FLAG_IMPASSABLE   0x02

// Characteristics of a node in the map, including distance from this node
//  to a/the destination.
//    Occupied: how many times the node has been traveled through.
//    Observed: how many times the node has been seen.
//    Boundary: how many times the node had an obstruction detection in it.
struct map_node_t
{
    // Position of node w/in grid.
    grid_coordinate_t pos;

    // Index of node that's one-closer to destination (-1 for no parent)
    grid_index_t parent_idx;
    // Direction toward destination. This is moving toward a node that's
    //  X-parents removed (e.g., 5)
    float direction_degs;

    float occupied;
    float observed;
    float boundary;

    map_node_flags_t flags;

    // Cost to traverse this node. This is computed as a function of the
    //  node's characteristics.
    float traversal_cost;

    // Total cost to reach destination. This is a unitless measure that
    //  includes biases to avoid heavily trafficed areas to encourage
    //  exploration
    float path_cost;

    void clear();
};


// The grid (map) itself.
class occupancy_grid_t
{
public:
    occupancy_grid_t(float node_width_meters, world_coordinate_t top_left,
        world_coordinate_t bottom_right);
    occupancy_grid_t(gaia::slam::area_map_t&);
    occupancy_grid_t(gaia::slam::working_map_t&);
    ~occupancy_grid_t();

    // Resets the map grid.
    void clear();

    // Returns a reference to tne map node at the specified location.
    map_node_t& get_node(float x_meters, float y_meters);
    map_node_flags_t& get_node_flags(float x_meters, float y_meters);

    // Apply sensor data to map from observation.
    void apply_sensor_data(const gaia::slam::observations_t&);

    void export_as_pnm(std::string file_name);

    // Path finding. Traces all routes to the destination. If map is
    //  embedded in larger map, the larger map is used to set boundary
    //  conditions for this map.
    void trace_routes(world_coordinate_t destination,
        occupancy_grid_t& parent_map);
    void trace_routes(world_coordinate_t destination);

protected:
    grid_index_t get_node_index(float pos_x_meters, float pos_y_meters);
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

    world_coordinate_t m_destination;

    ////////////////////////////////////////////////////////////////////
    // Path-finding. This is an algorithm derived from D*, ported from
    //  different project (used w/ permission).
    // FIFO queue for collecting list of which nodes need to be processed
    //  and either be assigned a cost to reach destination or update that
    //  cost. It's incorrectly referred to as a 'stack' sometimes.
    std::queue<grid_index_t> m_queue;

    // Add node to queue for future processing.
    void add_node_to_stack(const grid_index_t root_idx, 
        const node_offset_t offset, const float traverse_wt = 1.0f);

    // Check diagonal node to see if it should be added to queue, and
    //  do so if so.
    void add_node_to_stack_diag(const grid_index_t root_idx, 
        const node_offset_t offset);

    // Pop the next node off the stack and process it.
    void process_next_stack_node();

    // Returns approximate world location for this grid square.
    world_coordinate_t get_node_location(const grid_coordinate_t& pos);

    // Trace route(s) to destination. To each node, assigns approximate
    //  vector to reach destination.
    void compute_path_costs();

    // Setting destination. Also used for anchoring a node to a particular
    //  value, such as when setting boundary conditions.
    void add_anchor_to_path_stack(const grid_index_t idx, 
        const float path_weight);
};

} // namespace slam_sim

