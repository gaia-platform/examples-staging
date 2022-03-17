////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
#include <assert.h>
#include <math.h>

#include "occupancy.hpp"
#include "line_segment.hpp"

namespace slam_sim
{

using std::vector;

using utils::sensor_data_t;
using utils::landmark_description_t;
using utils::D2R;

////////////////////////////////////////////////////////////////////////
// Lookup table for caching sin/cos values

// Instead of computing sine and cosine all the time, precompute these
//  values for each range radial and cache it.
struct sin_cos_t {
    float s, c;
};

static sin_cos_t* s_sincos_lut = NULL;
static uint32_t s_sincos_lut_len = 0;


static void check_sincos_lut(sensor_data_t& data)
{
    if (s_sincos_lut_len == 0)
    {
        s_sincos_lut = new sin_cos_t[data.num_radials];
        // LUT isn't initialized. Do so now.
        double step_degs = 360.0 / data.num_radials;
        for (uint32_t i=0; i<data.num_radials; i++)
        {
            float theta_rads = D2R * step_degs * (float) i;
            // TODO Fall back to sin and cos if this isn't available.
            sincosf(theta_rads, &s_sincos_lut[i].s, &s_sincos_lut[i].c);
            //s_sincos_lut[i].s = sinf(D2R * theta_degs);
            //s_sincos_lut[i].c = cosf(D2R * theta_degs);
        }
        s_sincos_lut_len = data.num_radials;
    }
    // Verify LUT is of correct size.
    assert(s_sincos_lut_len == data.num_radials);
}

////////////////////////////////////////////////////////////////////////
// Constructors and destructors

occupancy_grid_t::occupancy_grid_t(float node_width_meters, float width_meters, 
    float height_meters)
{
    m_node_size_meters = node_width_meters;
    m_size.rows = (uint32_t) ceil(height_meters / node_width_meters);
    m_size.cols = (uint32_t) ceil(width_meters / node_width_meters);
    //
    uint32_t num_nodes = m_size.rows * m_size.cols;
    m_grid.resize(num_nodes);
    m_grid_flags.resize(num_nodes);

    clear();
}


////////////////////////////////////////////////////////////////////////
// Initialize map

void map_node_t::clear()
{
    this->occupied = 0;
    this->observed = 0;
    this->boundary = 0;
    this->landmarks = 0;

    this->traversal_cost = 0.0;
    this->distance = 0.0;
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


uint32_t occupancy_grid_t::get_node_index(float x_meters, float y_meters)
{
    uint32_t x_idx = (uint32_t) floor(m_size.cols / x_meters);
    if (x_idx >= m_size.cols)
    {
        x_idx = m_size.cols - 1;
    }
    uint32_t y_idx = (uint32_t) floor(m_size.rows / y_meters);
    if (y_idx >= m_size.rows)
    {
        y_idx = m_size.rows - 1;
    }
    return x_idx + y_idx * m_size.cols;
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

// Updates occupancy, oberved and boundary flags.
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
        float dist = (float) i * step_size_meters;
        float x_pos = pos_x_meters + dist * sc.s;
        float y_pos = pos_y_meters + dist * sc.c;
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


// Updates landmark flags.
void occupancy_grid_t::apply_landmarks(
    vector<landmark_description_t>& landmarks, 
    float pos_x_meters, float pos_y_meters)
{
    for (landmark_description_t& ld: landmarks)
    {
        map_node_flags_t& flags = get_node_flags(pos_x_meters + ld.x_meters, 
            pos_y_meters + ld.y_meters);
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


void occupancy_grid_t::apply_sensor_data(sensor_data_t& data, 
    float pos_x_meters, float pos_y_meters)
{
    check_sincos_lut(data);

    for (uint32_t i=0; i<data.num_radials; i++)
    {
        apply_radial(i, data.range_meters[i], pos_x_meters, pos_y_meters);
    }
    apply_landmarks(data.landmarks_visible, pos_x_meters, pos_y_meters);

    apply_flags();
}


// Applying sensor data to the map grids
////////////////////////////////////////////////////////////////////////
// Computer path weights


} // namespace slam_sim

