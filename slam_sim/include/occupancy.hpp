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

#include "map_types.hpp"
#include "sensor_data.hpp"

namespace slam_sim
{

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

// Indicates whether or not a node has been processed, if it's impassable
//  (i.e., a boundary was observed there) or if it's adjacent an 
//  impassable square)
#define PATH_NODE_FLAG_PROCESSED          0x01
#define PATH_NODE_FLAG_IMPASSABLE         0x02
#define PATH_NODE_FLAG_ADJ_IMPASSABLE     0x04
#define PATH_NODE_FLAG_CLOSE_IMPASSABLE   0x08
#define PATH_NODE_FLAG_NEAR_IMPASSABLE    0x10

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
  void count_bounds();

public:
    // Creates new uncached map.
    occupancy_grid_t(float node_width_meters, world_coordinate_t bottom_left,
        float width_meters, float height_meters);
    // Constructors to be called by rules.
    // Loads existing map, if present (map will be blank if it's doesn't
    //  exist yet).
    occupancy_grid_t(gaia::slam::area_map_t&);
    // Purges existing map and rebuilds a new one, using observed area as
    //  bounds.
    occupancy_grid_t(gaia::slam::area_map_t&, gaia::slam::observed_area_t&);

    ~occupancy_grid_t();

    // Resets the map grid.
    void clear();

    // Returns a reference to tne map node at the specified location.
    map_node_t& get_node(float x_meters, float y_meters);
    map_node_t& get_node(grid_index_t idx);
    map_node_flags_t& get_node_flags(float x_meters, float y_meters);
    map_node_flags_t& get_node_flags(grid_index_t idx);

    grid_index_t get_node_index(float pos_x_meters, float pos_y_meters);

    // Returns the center of the grid node at the specified coordinates.
    world_coordinate_t get_node_position(grid_coordinate_t& pos);

    // Apply sensor data to map from vertex.
    void apply_sensor_data(const gaia::slam::vertices_t&);

    void export_as_pnm(std::string file_name);

    // Path finding. Traces all routes to the destination. If map is
    //  embedded in larger map, the larger map is used to set boundary
    //  conditions for this map.
    void trace_routes(world_coordinate_t& destination,
        occupancy_grid_t& parent_map);
    void trace_routes(world_coordinate_t& destination);

    // Provide interface to blob ID in use so that can be pushed
    //  to the database.
    int32_t get_blob_id()                   { return m_blob_id;       }

    grid_size_t get_grid_size()             { return m_grid_size;     }
    world_coordinate_t get_bottom_left()    { return m_bottom_left;   }
    map_size_t get_map_size()               { return m_map_size;      }

protected:
    // Default constructor is used by subclasses. They're expected to
    //  do their own construction that's consistent and compatible with
    //  superclass. They really aren't subclasses in a traditional
    //  sense, as they don't extend the functionality of the base class,
    //  but they do represent different types, one each for different
    //  singleton map database tables, and this class is cleaner
    //  when they're moved onto their own.
    occupancy_grid_t() { }
    // Contructor helper function, with code common between constructors.
    void init(float node_width_meters, world_coordinate_t bottom_left,
        float width_meters, float height_meters);
    // Allocates own memory for m_grid, which is freed on destruction.
    // Grids created from DB blobs have memory for m_grid that is
    //  managed externally.
    void allocate_own_grid();

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

