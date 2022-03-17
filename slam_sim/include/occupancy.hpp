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

***********************************************************************/
#include <stdint.h>

#include <vector>

#include "sensor_data.hpp"
#include "landmark_description.hpp"

namespace slam_sim
{

// Characteristics of a node in the map, including distance from this node
//  to a/the destination.
// Occupancy: how many times the node has been travelled through.
// Observed: how many times the node has been seen.
// Boundary: how many times the node had an obstruction detection in it.
// Landmark: how many times the node had landmark reported in it.
struct map_node_t
{
    // Index of node that's one-closer to destination (-1 for no parent)
    int32_t parent_idx;
    // Direction toward destination. This is moving toward a node that's
    //  X-parents removed (e.g., 5)
    float direction_degs;

    
    uint32_t occupied;
    uint32_t observed;
    uint32_t boundary;
    uint32_t landmarks;

    // Cost to traverse this node. This is computed as a function of the
    //  node's characteristics.
    float traversal_cost;

    // Total cost to reach destination.
    float distance;


    void clear();
};


// Counter/flags for a node's characteristics when evaluating a sensor sweep.
struct map_node_flags_t
{
    uint8_t occupied;
    uint8_t observed;
    uint8_t boundary;
    uint8_t landmark;

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


class occupancy_grid_t
{
public:
    occupancy_grid_t(float node_width_meters, float width_meters, 
        float height_meters);
    ~occupancy_grid_t();

    // Initialization
    // Resets the map grid.
    void clear();
    // Initializes as inner map using outer map to set boundary conditions.
    // This is meant to be used to build a higher resolution local map to use
    //  based on a larger lower resolution area map.
    //void embed(const occupancy_grid_t& surrounding);

    //void resize(float width_meters, float height_meters);

    // Returns a reference to tne map node at the specified location.
    map_node_t& get_node(float x_meters, float y_meters);
    map_node_flags_t& get_node_flags(float x_meters, float y_meters);

    // Apply sensor data to map from position x,y.
    void apply_sensor_data(utils::sensor_data_t& data, float pos_x_meters,
        float pos_y_meters);

protected:
    uint32_t get_node_index(float pos_x_meters, float pos_y_meters);
    void apply_landmarks(
        std::vector<utils::landmark_description_t>& landmarks,
        float pos_x_meters, float pos_y_meters);
    void apply_radial(uint32_t radial, float range_meters, 
        float pos_x_meters, float pos_y_meters);
    void apply_flags();

    float m_node_size_meters;
    grid_size_t m_size;

    std::vector<map_node_t> m_grid;
    std::vector<map_node_flags_t> m_grid_flags;

    world_coordinate_t m_destination;
};


//////////////////////////////////////////////////////////////////////////
//// API
//
//// Usage: initialize the map, apply sensor data to it, add destinations,
////  and then compile. Map nodes will have vectors to move toward
////  destination.
//
//void reset_map(occupancy_grid_t& map);
//// Initializes inner map using outer map to set boundary conditions. This
////  is meant to be used to build a higher resolution local map to use
////  based on a larger lower resolution area map.
//void embed_map(const occupancy_grid_t& outer, occupancy_grid_t& inner);
//
//// Apply sensor data to it.
//void apply_sensor_data_to_map(occupancy_grid_t& map, sensor_data_t& data);
//
//// Add destination to the map.
//void add_destination(occupancy_grid_t& map, double x_meters, y_meters);
//
//// Build distances to destination(s) and directional vectors.
//void compile_map(occupancy_grid_t& map);
//
//// Saves an image of the map as a .pnm file.
//void export_as_pnm(const occupancy_grid_t& map, string file_name);

} // namespace slam_sim

