////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
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

using utils::sensor_data_t;
using utils::landmark_description_t;
using utils::D2R;

using gaia::slam::landmark_sightings_t;
using gaia::slam::observations_t;

////////////////////////////////////////////////////////////////////////
// Lookup table for caching sin/cos values

// Instead of computing sine and cosine all the time, precompute these
//  values for each range radial and cache it.
struct sin_cos_t {
    float s, c;
};

static sin_cos_t* s_sincos_lut = NULL;
static uint32_t s_sincos_lut_len = 0;


static void check_sincos_lut(uint32_t num_radials)
{
    if (s_sincos_lut_len == 0)
    {
        s_sincos_lut = new sin_cos_t[num_radials];
        // LUT isn't initialized. Do so now.
        double step_degs = 360.0 / num_radials;
        for (uint32_t i=0; i<num_radials; i++)
        {
            float theta_rads = D2R * step_degs * (float) i;
            // TODO Fall back to sin and cos if this isn't available.
            sincosf(theta_rads, &s_sincos_lut[i].s, &s_sincos_lut[i].c);
            //s_sincos_lut[i].s = sinf(D2R * theta_degs);
            //s_sincos_lut[i].c = cosf(D2R * theta_degs);
        }
        s_sincos_lut_len = num_radials;
    }
    // Verify LUT is of correct size.
    assert(s_sincos_lut_len == num_radials);
}

////////////////////////////////////////////////////////////////////////
// Constructors and destructors

occupancy_grid_t::occupancy_grid_t(float node_width_meters, float width_meters, 
    float height_meters)
{
    m_node_size_meters = node_width_meters;
    m_grid_size.rows = (uint32_t) ceil(height_meters / node_width_meters);
    m_grid_size.cols = (uint32_t) ceil(width_meters / node_width_meters);
    m_map_size.x_meters = width_meters;
    m_map_size.y_meters = height_meters;
    m_top_left.x_meters = -width_meters / 2.0;
    m_bottom_right.x_meters = width_meters / 2.0;
    m_top_left.y_meters = -height_meters / 2.0;
    m_bottom_right.y_meters = height_meters / 2.0;
    //
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    m_grid.resize(num_nodes);
    m_grid_flags.resize(num_nodes);
    clear();
}


////////////////////////////////////////////////////////////////////////
// Initialize map

void map_node_t::clear()
{
    this->parent_idx = -1;
    this->direction_degs = 0.0f;

    this->occupied = 0.0f;
    this->observed = 0.0f;
    this->boundary = 0.0f;
    this->landmarks = 0.0f;

    this->traversal_cost = 0.0f;
    this->distance = 0.0f;
}


void map_node_flags_t::clear()
{
    this->occupied = 0;
    this->observed = 0;
    this->boundary = 0;
    this->landmark = 0;
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
    float left_inset_meters = x_meters - m_top_left.x_meters;
    uint32_t x_idx = (uint32_t) floor(left_inset_meters / m_map_size.x_meters);
    if (x_idx >= m_grid_size.cols)
    {
        x_idx = m_grid_size.cols - 1;
    }
    float bottom_inset_meters = y_meters - m_bottom_right.y_meters;
    uint32_t y_idx = 
        (uint32_t) floor(bottom_inset_meters / m_map_size.y_meters);
    if (y_idx >= m_grid_size.rows)
    {
        y_idx = m_grid_size.rows - 1;
    }
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
void occupancy_grid_t::apply_radial(uint32_t radial, float range_meters,
    float pos_x_meters, float pos_y_meters)
{
    // Set occupancy for this grid square.
    map_node_flags_t& home_flags = get_node_flags(pos_x_meters, pos_y_meters);
    home_flags.occupied = 1;

    // Set observed and boundary flags.
    // Measured distance on radial.
    double dist_meters = range_meters < 0.0 
        ? utils::RANGE_SENSOR_MAX_METERS : range_meters;
    
    const sin_cos_t& sc = s_sincos_lut[radial];
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
        float x_pos = dist * sc.s - pos_x_meters;
        float y_pos = dist * sc.c - pos_y_meters;
        map_node_flags_t& flags = get_node_flags(x_pos, y_pos);
        flags.observed = 1;
        // If this is the end of the radial and a range was detected,
        //  mark the boundary flag.
        if ((i == num_steps) && (range_meters > 0.0))
        {
printf("  Boundary on %d at %.2f,%.2f (%.2f, %.2f, %.2f)\n", radial, x_pos, y_pos, pos_x_meters, pos_y_meters, dist);
            flags.boundary = 1;
        }
    }
}


// Updates landmark flags.
void occupancy_grid_t::apply_landmarks(const observations_t& obs)
{
    float pos_x_meters = obs.pos_x_meters();
    float pos_y_meters = obs.pos_y_meters();
    for (const landmark_sightings_t& ls: obs.landmark_sightings())
    {
        int32_t radial = (int32_t) floor(obs.num_radials() 
            * ls.bearing_degs() / 360.0);
        assert(radial >= 0);
        assert(radial < obs.num_radials());
        const sin_cos_t& sc = s_sincos_lut[radial];
        float dx_meters = ls.range_meters() * sc.s;
        float dy_meters = ls.range_meters() * sc.c;

        map_node_flags_t& flags = get_node_flags(pos_x_meters + dx_meters, 
            pos_y_meters + dy_meters);
        flags.landmark = 1;
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
        node.landmarks += flags.landmark;
    }
}


void occupancy_grid_t::apply_sensor_data(const observations_t& obs)
{
    check_sincos_lut(obs.num_radials());
printf("Applying sensor data at %.2f,%.2f (%d)\n", obs.pos_x_meters(), obs.pos_y_meters(), obs.id());
    float pos_x_meters = obs.pos_x_meters();
    float pos_y_meters = obs.pos_y_meters();
    for (int32_t i=0; i<obs.num_radials(); i++)
    {
        apply_radial(i, obs.distance_meters()[i], pos_x_meters, pos_y_meters);
    }
    apply_landmarks(obs);

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
                float landmarks = node.landmarks / node.observed;
                assert(landmarks <= 1.0);
                b[idx] = (uint8_t) round(255.0 * landmarks);
            }
            g[idx] = node.occupied > 0.0 ? 128 : 0;
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

