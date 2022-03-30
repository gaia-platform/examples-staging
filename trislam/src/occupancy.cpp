////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
#include <assert.h>
#include <math.h>

#include <fstream>
#include <iostream>

#include "line_segment.hpp"
#include "occupancy.hpp"
#include "slam_sim.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using std::cerr;

using gaia::slam::observations_t;
using gaia::slam::positions_t;
using gaia::slam::range_data_t;


////////////////////////////////////////////////////////////////////////
// Constructors and destructors

occupancy_grid_t::occupancy_grid_t(float node_width_meters, 
    world_coordinate_t top_left, world_coordinate_t bottom_right)
{
    float width_meters = bottom_right.x_meters - top_left.x_meters;
    float height_meters = top_left.y_meters - bottom_right.y_meters;
    // Number of grids in map.
    m_grid_size.rows = (uint32_t) ceil(height_meters / node_width_meters);
    m_grid_size.cols = (uint32_t) ceil(width_meters / node_width_meters);
    // Physcal size (width, height) of map grids.
    m_node_size_meters = node_width_meters;
    // Physical size of map.
    m_map_size.x_meters = width_meters;
    m_map_size.y_meters = height_meters;
    // Map center.
    m_bottom_left.x_meters = top_left.x_meters;
    m_bottom_left.y_meters = bottom_right.y_meters;
    //
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    m_grid.resize(num_nodes);
    m_grid_flags.resize(num_nodes);
    clear();
}

occupancy_grid_t::~occupancy_grid_t()
{
}

////////////////////////////////////////////////////////////////////////
// Initialize map

void map_node_t::clear()
{
    this->occupied = 0.0f;
    this->observed = 0.0f;
    this->boundary = 0.0f;
}


void map_node_flags_t::clear()
{
    this->occupied = 0;
    this->observed = 0;
    this->boundary = 0;
}


void occupancy_grid_t::clear()
{
    for (uint32_t i=0; i<m_grid.size(); i++)
    {
        m_grid[i].clear();
    }
    for (uint32_t i=0; i<m_grid_flags.size(); i++)
    {
        m_grid_flags[i].clear();
    }
}


////////////////////////////////////////////////////////////////////////
// Node access


// Returns index of map node at specified location. Location uses 
uint32_t occupancy_grid_t::get_node_index(float x_meters, float y_meters)
{
    // x index.
    float left_inset_meters = x_meters - m_bottom_left.x_meters;;
    uint32_t x_idx = (uint32_t) floor(left_inset_meters / m_node_size_meters);
    if (x_idx >= m_grid_size.cols)
    {
        x_idx = m_grid_size.cols - 1;
    }
    // y index.
    float bottom_inset_meters = y_meters - m_bottom_left.y_meters;;
    uint32_t y_idx = 
        (uint32_t) floor(bottom_inset_meters / m_node_size_meters);
    if (y_idx >= m_grid_size.rows)
    {
        y_idx = m_grid_size.rows - 1;
    }
//printf("  %.2f,%.2f at idcs %d,%d (of %d,%d = %d)\n", x_meters, y_meters, x_idx, y_idx, m_grid_size.cols, m_grid_size.rows, x_idx+y_idx*m_grid_size.cols);
    return x_idx + y_idx * m_grid_size.cols;
}


map_node_t& occupancy_grid_t::get_node(float x_meters, float y_meters)
{
    uint32_t idx = get_node_index(x_meters, y_meters);
    return m_grid[idx];
}


map_node_flags_t& occupancy_grid_t::get_node_flags(
    float x_meters, float y_meters)
{
    uint32_t idx = get_node_index(x_meters, y_meters);
    return m_grid_flags[idx];
}


////////////////////////////////////////////////////////////////////////
// Applying sensor data to the map grids

// Updates occupancy, oberved and boundary flags in map from 
//  observations on this radial.
// Steps through radial and estimates what grid squares it crosses.
void occupancy_grid_t::apply_radial(float radial_degs, float range_meters,
    float pos_x_meters, float pos_y_meters)
{
    // Set occupancy for this grid square.
    map_node_flags_t& home_flags = get_node_flags(pos_x_meters, pos_y_meters);
//printf("OCCUPANCY position %.2f,%.2f\n", pos_x_meters, pos_y_meters);
    home_flags.occupied = 1;

    // Set observed and boundary flags.
    // Measured distance on radial.
    double dist_meters = range_meters < 0.0 
        ? c_range_sensor_max_meters : range_meters;
    
    float s, c;
    sincosf(c_deg_to_rad * radial_degs, &s, &c);
    // Number of points on radial to examine. Sample at a fraction of 
    //  the grid size.
    uint32_t num_steps = (uint32_t) 
        ceil(dist_meters / (m_node_size_meters / 3.0));
    double step_size_meters = dist_meters / num_steps;
    for (uint32_t i=1; i<=num_steps; i++)
    {
        // Get position in world map, adjusting relative range data by
        //  bot's absolute position.
        float dist = (float) i * step_size_meters;
        float x_pos = dist * s + pos_x_meters;
        float y_pos = dist * c + pos_y_meters;
        map_node_flags_t& flags = get_node_flags(x_pos, y_pos);
        flags.observed = 1;
        // If this is the end of the radial and a range was detected,
        //  mark the boundary flag.
        if ((i == num_steps) && (range_meters > 0.0))
        {
            flags.boundary = 1;
        }
    }
}


// Increments map_node_t values from contents of map_node_flags_t.
void occupancy_grid_t::apply_flags()
{
    for (uint32_t i=0; i<m_grid.size(); i++)
    {
        map_node_flags_t& flags = m_grid_flags[i];
        map_node_t& node = m_grid[i];

        node.occupied += flags.occupied;
        node.observed += flags.observed;
        node.boundary += flags.boundary;
    }
}


void occupancy_grid_t::apply_sensor_data(const observations_t& obs)
{
    positions_t pos = obs.position();
    range_data_t r = obs.range_data();

//printf("Applying sensor data at %.2f,%.2f (%d)\n", pos.pos_x_meters(), pos.pos_y_meters(), obs.id());
    for (int32_t i=0; i<r.num_radials(); i++)
    {
        apply_radial(r.bearing_degs()[i], r.distance_meters()[i], 
            pos.x_meters(), pos.y_meters());
    }

    apply_flags();
}


// Applying sensor data to the map grids
////////////////////////////////////////////////////////////////////////
// Compute path weights

////////////////////////////////////////////////////////////////////////
// Export map

void occupancy_grid_t::export_as_pnm(string file_name)
{
    uint32_t n_pix = m_grid_size.cols * m_grid_size.rows;
    uint8_t* r = new uint8_t[n_pix];
    uint8_t* g = new uint8_t[n_pix];
    uint8_t* b = new uint8_t[n_pix];
    // Create image.
    for (uint32_t y=0; y<m_grid_size.rows; y++)
    {
        for (uint32_t x=0; x<m_grid_size.cols; x++)
        {
            uint32_t idx = x + y * m_grid_size.cols;
            map_node_t& node = m_grid[idx];
            if (node.observed > 0.0)
            {
                float boundary = node.boundary / node.observed;
                assert(boundary <= 1.0);
                r[idx] = (uint8_t) round(255.0 * boundary);
            }
            g[idx] = node.occupied > 0.0 ? 255 : 0;
        }
    }


    // Export image.
    try 
    {
        std::ofstream f(file_name, std::ofstream::binary);
        f << "P6\n# Slam map\n" << m_grid_size.cols << " " 
            << m_grid_size.rows << "\n255\n";
        for (uint32_t i=0; i<n_pix; i++)
        {
            f << r[i] << g[i] << b[i];
        }

    }
    catch (const std::exception& ioe)
    {
        cerr << "Error writing to " << file_name << ": " << ioe.what() << "\n";
    }
    delete[] r;
    delete[] g;
    delete[] b;
}


} // namespace slam_sim

