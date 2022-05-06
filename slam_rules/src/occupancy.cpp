////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Logic for making/updating binary occupancy grid.
//
////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <math.h>

#include <fstream>
#include <iostream>

#include <gaia/logger.hpp>

#include "blob_cache.hpp"
#include "constants.hpp"
#include "globals.hpp"
#include "line_segment.hpp"
#include "occupancy.hpp"
#include "slam_sim.hpp"
#include "txn.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using std::cerr;

using gaia::slam::vertices_t;
using gaia::slam::positions_t;
using gaia::slam::range_data_t;

using gaia::slam::area_map_t;;
//using gaia::slam::working_map_t;;
using gaia::slam::observed_area_t;;

using gaia::slam::area_map_writer;

blob_cache_t g_area_blobs;
blob_cache_t g_working_blobs;


////////////////////////////////////////////////////////////////////////
// Construction and destruction

void occupancy_grid_t::init(float node_width_meters,
    world_coordinate_t bottom_left, float width_meters, float height_meters)
{
    // Number of grids in map.
    m_grid_size.rows = (uint32_t) ceil(height_meters / node_width_meters);
    m_grid_size.cols = (uint32_t) ceil(width_meters / node_width_meters);
    // Physcal size (width, height) of map grids.
    m_node_size_meters = node_width_meters;
    // Physical size of map.
    m_map_size.x_meters = width_meters;
    m_map_size.y_meters = height_meters;
    // Map placement.
    m_bottom_left = bottom_left;
}


void occupancy_grid_t::allocate_own_grid()
{
    // Signal that 'this' owns the memory allocation.
    m_blob_id = -1;
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    m_grid = (map_node_t*) malloc(num_nodes * sizeof *m_grid);
}


// Creates new uncached map.
occupancy_grid_t::occupancy_grid_t(float node_width_meters,
    world_coordinate_t bottom_left, float width_meters, float height_meters)
{
    init(node_width_meters, bottom_left, width_meters, height_meters);
    allocate_own_grid();
    clear();
}


// Loads existing map.
occupancy_grid_t::occupancy_grid_t(area_map_t& am)
{
    world_coordinate_t bottom_left = {
        .x_meters = am.left_meters(),
        .y_meters = am.bottom_meters()
    };
    init(c_area_map_node_width_meters, bottom_left,
        am.right_meters() - am.left_meters(),
        am.top_meters() - am.bottom_meters());
    // Recover memory from blob cache.
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    m_blob_id = am.blob_id();;
    // Fetch existing blob.
    blob_t* blob = g_area_blobs.get_blob(m_blob_id);
    if (blob == NULL)
    {
        // Blob doesn't exist yet. This means that the process was
        //  started up against an existing database. This shouldn't
        //  happen, but allow it. Create it.
        gaia_log::app().warn("Area map with ID {} did not have cached "
            "blob. Recreating one.", m_blob_id);
        size_t sz = num_nodes * sizeof *m_grid;
        blob = g_area_blobs.create_blob(m_blob_id, sz);
        m_grid = (map_node_t*) blob->data();
        // New grid. Nodes need initialization.
        clear();
    } else {
        assert(m_grid_size.rows == am.num_rows());
        assert(m_grid_size.cols == am.num_cols());
        m_grid = (map_node_t*) blob->data();
        // Existing grid. Should be initialized already.
    }
}


// Overwrites existing cached area map.
occupancy_grid_t::occupancy_grid_t(area_map_t& am, observed_area_t& area)
{
    world_coordinate_t bottom_left = {
        .x_meters = am.left_meters(),
        .y_meters = am.bottom_meters()
    };
    init(c_area_map_node_width_meters, bottom_left,
        area.right_meters() - area.left_meters(),
        area.top_meters() - area.bottom_meters());
    // Get rid of old content and create a new empty blob.
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    uint32_t old_blob_id = am.blob_id();
    size_t sz = num_nodes * sizeof *m_grid;
    m_blob_id = old_blob_id + 1;
    blob_t* blob = g_area_blobs.create_blob(m_blob_id, sz, old_blob_id);
    m_grid = (map_node_t*) blob->data();
    clear();
    // Store changes in DB.
    area_map_writer writer = am.writer();
    writer.blob_id = m_blob_id;
    writer.left_meters = area.left_meters();
    writer.right_meters = area.right_meters();
    writer.top_meters = area.top_meters();
    writer.bottom_meters = area.bottom_meters();
    writer.num_rows = m_grid_size.rows;
    writer.num_cols = m_grid_size.cols;
    writer.update_row();
}


occupancy_grid_t::~occupancy_grid_t()
{
    if (m_blob_id < 0)
    {
        // Memory is owned by this.
        free(m_grid);
    }
}


////////////////////////////////////////////////////////////////////////
// Initialize map

void map_node_t::clear()
{
    this->parent_idx.idx = c_invalid_grid_idx;
    this->direction_degs = 0.0f;

    this->occupied = 0.0f;
    this->observed = 0.0f;
    this->boundary = 0.0f;

    this->flags.clear();

    this->traversal_cost = 1.0f;
    this->path_cost = 0.0f;
}


void map_node_flags_t::clear()
{
    this->occupied = 0;
    this->observed = 0;
    this->boundary = 0;
    this->state = 0;
}


void occupancy_grid_t::clear()
{
    for (uint32_t y=0; y<m_grid_size.rows; y++)
    {
        for (uint32_t x=0; x<m_grid_size.cols; x++)
        {
            uint32_t idx = x + y * m_grid_size.cols;
            m_grid[idx].clear();
            m_grid[idx].pos.x = x;
            m_grid[idx].pos.y = y;
        }
    }
}


////////////////////////////////////////////////////////////////////////
// Node access


// Returns index of map node at specified location. Location uses 
grid_index_t occupancy_grid_t::get_node_index(float x_meters, float y_meters)
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
    grid_index_t idx = { .idx = x_idx + y_idx * m_grid_size.cols };
    return idx;
}


map_node_t& occupancy_grid_t::get_node(float x_meters, float y_meters)
{
    grid_index_t idx = get_node_index(x_meters, y_meters);
    return m_grid[idx.idx];
}


map_node_t& occupancy_grid_t::get_node(grid_index_t idx)
{
    return m_grid[idx.idx];
}


map_node_flags_t& occupancy_grid_t::get_node_flags(
    float x_meters, float y_meters)
{
    grid_index_t idx = get_node_index(x_meters, y_meters);
    return m_grid[idx.idx].flags;
}


map_node_flags_t& occupancy_grid_t::get_node_flags(grid_index_t idx)
{
    return m_grid[idx.idx].flags;
}


world_coordinate_t occupancy_grid_t::get_node_position(grid_coordinate_t& pos)
{
    assert(pos.x < m_grid_size.cols);
    assert(pos.y < m_grid_size.rows);
    world_coordinate_t coord;
    coord.x_meters = m_bottom_left.x_meters 
        + ((float) pos.x + 0.5f) * m_node_size_meters;
    coord.y_meters = m_bottom_left.y_meters + m_map_size.y_meters
        - (((float) pos.y + 0.5f) * m_node_size_meters);
    return coord;
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
    uint32_t num_nodes = m_grid_size.rows * m_grid_size.cols;
    for (uint32_t i=0; i<num_nodes; i++)
    {
        map_node_flags_t& flags = m_grid[i].flags;
        map_node_t& node = m_grid[i];
        node.occupied += flags.occupied;
        node.observed += flags.observed;
        node.boundary += flags.boundary;
        // Set 'inpassable' flag if boundary was observed in this node.
        if (node.boundary > 0.0)
        {
            // Make an exception if it's rarely observed, as it could have
            //  been a measurement error.
            if (flags.boundary != 0)
            {
                // Boundary was just seen. Observe it regardless of any
                //  flukes in the past.
                node.flags.state |= PATH_NODE_FLAG_IMPASSABLE;
            }
            else if (node.boundary / node.observed > 0.1)
            {
                assert(node.observed > 0.0);
                node.flags.state |= PATH_NODE_FLAG_IMPASSABLE;
            }
        }
        // Mark impassable's neighbors as near impassable or close to
        //  impassable.
        if (node.flags.state & PATH_NODE_FLAG_IMPASSABLE)
        {
            // First 8 deltas are in first ring around node. Those
            //  are ADJ_IMPASSABLE. Second ring are CLOSE_IMPASSABLE.
            //  Third ring are NEAR_IMPASSABLE.
            const int32_t x_delta[48] = { 
                -1,  0,  1, -1,  1, -1,  0,  1,
                -2, -1,  0,  1,  2, -2,  2, -2, 
                 2, -2,  2, -2, -1,  0,  1,  2,
                -3, -2, -1,  0,  1,  2,  3,
                -3,  3, -3,  3, -3,  3, -3,  3, -3,  3,
                -3, -2, -1,  0,  1,  2,  3
            };
            const int32_t y_delta[48] = {
                -1, -1, -1,  0,  0,  1,  1,  1,
                -2, -2, -2, -2, -2, -1, -1,  0, 
                 0,  1,  1,  2,  2,  2,  2,  2,
                -3, -3, -3, -3, -3, -3, -3, 
                -2, -2, -1, -1,  0,  0,  1,  1,  2,  2,
                 3, -3,  3,  3,  3,  3,  3  
            };
            for (uint32_t i=0; i<48; i++)
            {
                uint32_t nbr_x = (uint32_t) 
                    ((int32_t) node.pos.x + x_delta[i]);
                uint32_t nbr_y = (uint32_t) 
                    ((int32_t) node.pos.y + y_delta[i]);
                if ((nbr_x < m_grid_size.cols) && (nbr_y < m_grid_size.rows))
                {
                    uint32_t nbr_idx = nbr_x + nbr_y * m_grid_size.cols;
                    if (i < 8)
                    {
                        m_grid[nbr_idx].flags.state |= 
                            PATH_NODE_FLAG_ADJ_IMPASSABLE;
                    }
                    else if (i < 24)
                    {
                        m_grid[nbr_idx].flags.state |= 
                            PATH_NODE_FLAG_CLOSE_IMPASSABLE;
                    }
                    else 
                    {
                        m_grid[nbr_idx].flags.state |= 
                            PATH_NODE_FLAG_NEAR_IMPASSABLE;
                    }
                }
            }
        }
    }
}


void occupancy_grid_t::apply_sensor_data(const vertices_t& obs)
{
    // TODO make sure that sensor data from outside of map is ignored.
    //
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
// See path_map.cpp

////////////////////////////////////////////////////////////////////////
// Export map

void occupancy_grid_t::export_as_pnm(string file_name)
{
    uint32_t n_pix = m_grid_size.cols * m_grid_size.rows;
    std::vector<uint8_t> r(n_pix, 0), g(n_pix, 0), b(n_pix, 0);
//printf("Exporting 0x%08lx  %d\n", (uint64_t) m_grid, m_blob_id);

//printf("-----------------------------------------\n");
    // Draw position and boundaries.
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
    // Indicate destination.
    txn_t txn;
    txn.begin();
    uint32_t which_dest = (g_next_destination + g_destinations.size() - 1)
        % g_destinations.size();
    world_coordinate_t destination = g_destinations[which_dest];
    const map_node_t& dest = 
        get_node(destination.x_meters, destination.y_meters);
    uint32_t dest_idx = dest.pos.x + dest.pos.y * m_grid_size.cols;
    txn.commit();
    b[dest_idx] = 255;
    g[dest_idx] = 128;

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
}

//void occupancy_grid_t::count_bounds()
//{
//    uint32_t cnt = 0;
//    for (uint32_t y=0; y<m_grid_size.rows; y++)
//    {
//        for (uint32_t x=0; x<m_grid_size.cols; x++)
//        {
//            uint32_t idx = x + y * m_grid_size.cols;
//            if (m_grid[idx].boundary > 0.0f)
//            {
////printf("%d,%d  (%dx%d=%d)  bounds %f\n", x, y, m_grid_size.cols, m_grid_size.rows, idx, m_grid[idx].boundary);
//                cnt++;
//            }
//        }
//    }
//    printf("Boundary count: %d\n", cnt);
//}

} // namespace slam_sim
