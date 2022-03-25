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
//  landmarks (e.g., 'tables').
//
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <mutex>
#include <vector>

#include <gaia/logger.hpp>

#include "json.hpp"
#include "line_segment.hpp"
#include "sensor_data.hpp"
#include "slam_sim.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using nlohmann::json;

using utils::line_segment_t;
using utils::sensor_data_t;

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
            double x0 = p0["x"];
            double y0 = p0["y"];
            const json& p1 = vertices[j+1];
            double x1 = p1["x"];
            double y1 = p1["y"];
            g_world_lines.push_back(line_segment_t(x0, y0, x1, y1));
        }
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
static void calculate_range_data(double x_meters, double y_meters, 
    double heading_degs, sensor_data_t& data)
{
    data.range_meters.clear();
    data.num_radials = NUM_RANGE_RADIALS;
//printf("RANGES from %.3f,%.3f\n", x_meters, y_meters);
    double step_degs = RANGE_SENSOR_SWEEP_DEGS / (NUM_RANGE_RADIALS - 1);
    // Get range on each radial, and store both distance and radial degs.
    for (uint32_t n=0; n<NUM_RANGE_RADIALS; n++)
    {
        // Get this radial and constrain to [0,360)
        double theta_degs = heading_degs - RANGE_SENSOR_SWEEP_DEGS
            + (double) n * step_degs;
        theta_degs = theta_degs >= 360.0 ? theta_degs - 360.0 : theta_degs;
        theta_degs = theta_degs < 0.0    ? theta_degs + 360.0 : theta_degs;
        // Measure distance on this radial
        double min_meters = -1.0;
        int32_t line_num = -1;
        for (uint32_t i=0; i<g_world_lines.size(); i++)
        {
            line_segment_t& seg = g_world_lines[i];
            double dist_meters =
                seg.intersect_range(x_meters, y_meters, theta_degs);
            if (dist_meters > 0.0)
            {
                if ((min_meters < 0.0) || (dist_meters < min_meters))
                {
                    min_meters = dist_meters;
                    line_num = i;
                }
            }
        }
//printf("   %3d   %8.2f   %6.2f    %4d\n", n, theta_degs, min_meters, line_num);
        if (min_meters > RANGE_SENSOR_MAX_METERS)
        {
            min_meters = -1.0;
        }
        data.range_meters.push_back(min_meters);
        data.bearing_degs.push_back(theta_degs);
    }
}

} // namespace slam_sim

