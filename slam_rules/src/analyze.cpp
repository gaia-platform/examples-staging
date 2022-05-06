////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// This file calculates the sensor view of the environment. The
//  environment is made up of a 2D map (flatland) that has walls and
//  objects (e.g., 'tables').
//
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <mutex>
#include <vector>

#include <gaia/logger.hpp>

#include "constants.hpp"
#include "json.hpp"
#include "globals.hpp"
#include "line_segment.hpp"
#include "map_types.hpp"
#include "sensor_data.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using nlohmann::json;

// Line segments describing outline of all objects in the world.
vector<line_segment_t> g_world_lines;


// Parse json map data and copy to vectors describing the world. Assume
//  that the json format is correct.
static void set_map(const char* map)
{
    const json& world_map = json::parse(map);
    // copy map data into line segment array
    const json& world_objects = world_map["world"];
    for (uint32_t i=0; i<world_objects.size(); i++)
    {
        const json& vertices = world_objects[i]["vertices"];;
        for (uint32_t j=0; j<vertices.size()-1; j++)
        {
            const json& p0 = vertices[j];
            float x0 = p0["x"];
            float y0 = p0["y"];
            const json& p1 = vertices[j+1];
            float x1 = p1["x"];
            float y1 = p1["y"];
            g_world_lines.push_back(line_segment_t(x0, y0, x1, y1));
        }
    }
    const json& destinations = world_map["destinations"];
    for (uint32_t i=0; i<destinations.size(); i++)
    {
        const json& point = destinations[i];
        world_coordinate_t dest;
        dest.x_meters = point["x"];
        dest.y_meters = point["y"];
        g_destinations.push_back(dest);
    }
}


void load_world_map(const char* world_map)
{
    FILE* fp = fopen(world_map, "r");
    if (!fp)
    {
        fprintf(stderr, "Failed to open %s\n", world_map);
        exit(1);
    }
    fseek(fp, 0, SEEK_END);
    size_t len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char* buf = new char[len+1];
    if (fread(buf, len, 1, fp) != 1)
    {
        fprintf(stderr, "Failed to read world map %s\n", world_map);
        exit(1);
    }
    fclose(fp);

    buf[len] = 0;
    set_map(buf);
    free(buf);
}


// For each observation radial, check point of intersection of that
//  radial with the nearest positive intersection with the line segments
//  making up the 2D world map.
void calculate_range_data(map_coord_t& coord, sensor_data_t& data)
{
//printf("Calculating range data from %f,%f\n", coord.x_meters, coord.y_meters);
    data.range_meters.clear();
    data.bearing_degs.clear();
    float step_degs = c_range_sensor_sweep_degs / (c_num_range_radials - 1);
    // Get range on each radial, and store both distance and radial degs.
    for (uint32_t n=0; n<c_num_range_radials; n++)
    {
        // Get this radial and constrain to [0,360)
        float theta_degs = coord.heading_degs - c_range_sensor_sweep_degs/2.0
            + (float) n * step_degs;
        theta_degs = theta_degs >= 360.0 ? theta_degs - 360.0 : theta_degs;
        theta_degs = theta_degs < 0.0    ? theta_degs + 360.0 : theta_degs;
//printf("  %d: %.1f\n", n, theta_degs);
        // Measure distance on this radial
        float min_meters = -1.0;
        int32_t line_num = -1;
        for (uint32_t i=0; i<g_world_lines.size(); i++)
        {
            line_segment_t& seg = g_world_lines[i];
            float dist_meters =
                seg.intersect_range(coord.x_meters, coord.y_meters, theta_degs);
//printf("    %.1f,%.1f -> %.1f,%.1f   range %.2f\n", seg.m_x0, seg.m_y0, seg.m_x1, seg.m_y1, dist_meters);
            if (dist_meters > 0.0)
            {
                if ((min_meters < 0.0) || (dist_meters < min_meters))
                {
                    min_meters = dist_meters;
                    line_num = i;
                }
            }
        }
        if (min_meters > c_range_sensor_max_meters)
        {
            min_meters = -1.0;
        }
//printf("    range %.1f at %.1f\n", min_meters, theta_degs);
        data.range_meters.push_back(min_meters);
        data.bearing_degs.push_back(theta_degs);
    }
}

} // namespace slam_sim

