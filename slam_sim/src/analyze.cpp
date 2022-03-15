////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
//
// A very simple environment simulation interface.
// 
// This file calculates the sensor view of the environment. The 
//  environment is made up of a 2D map (flatland) that has walls and
//  explicit landmarks. In SLAM, landmarks will typically be calculated
//  based on aligning salient features between different observations.
//  In this case, such alignment is assumed to have been done, as the
//  point of the example here is what to do when you find a landmark,
//  not feature extraction and mapping. A more intuitive way to think 
//  about landmarks in this example, as its implemented, is that 
//  they're QR codes on a wall.
//
////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <string>
#include <mutex>
#include <vector>

#include "json.hpp"
#include "line_segment.hpp"
#include "sensor_data.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using nlohmann::json;

using utils::line_segment_t;
using utils::landmark_description_t;
using utils::sensor_data_t;

constexpr int32_t NUM_RANGE_RADIALS = 180;

// Line segments describing outline of all objects in the world.
vector<line_segment_t> g_world_lines;
vector<landmark_description_t> g_landmarks;


// Parse json map data and copy to vectors describing the world. Assume 
//  that the json format is correct.
static void set_map(const char* map)
{
    const json& world_map = json::parse(map);
    // copy map data into line segment array
    const json& world_objects = world_map["world"];
    for (uint32_t i=0; i<world_objects.size(); i++) {
        const json& vertices = world_objects[i]["vertices"];;
        for (uint32_t j=0; j<vertices.size()-1; j++) {
            const json& p0 = vertices[j];
            double x0 = p0["x"];
            double y0 = p0["y"];
            const json& p1 = vertices[j+1];
            double x1 = p1["x"];
            double y1 = p1["y"];
            g_world_lines.push_back(line_segment_t(x0, y0, x1, y1));
        }
    }

    // copy landmark data
    const json& land_objects = world_map["landmarks"];
    for (uint32_t i=0; i<land_objects.size(); i++) {
        string name = land_objects[i]["name"];
        int32_t id = land_objects[i]["id"];
        double x_meters = land_objects[i]["x"];
        double y_meters = land_objects[i]["y"];
        g_landmarks.push_back(
            landmark_description_t(name, id, x_meters, y_meters));
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


// Load map
////////////////////////////////////////////////////////////////////////
// Build sensor view


static void calculate_ranges(double x_meters, double y_meters, 
    sensor_data_t& data)
{
    data.range_meters.clear();
    data.num_radials = NUM_RANGE_RADIALS;
    for (uint32_t n=0; n<NUM_RANGE_RADIALS; n++)
    {
        double theta_degs = n * 360.0 / (double) NUM_RANGE_RADIALS;
        double min_meters = -1.0;
        for (uint32_t i=0; i<g_world_lines.size(); i++)
        {
            line_segment_t& seg = g_world_lines[i];
            double dist_meters = 
                seg.intersect_range(x_meters, y_meters, theta_degs);
            if (dist_meters > 0.0) {
                if ((min_meters < 0.0) || (dist_meters < min_meters)) {
                    min_meters = dist_meters;
                }
            }
        }
        if (min_meters > utils::RANGE_SENSOR_MAX_METERS)
        {
            min_meters = -1.0;
        }
        data.range_meters.push_back(min_meters);
    }
}


static void calculate_landmarks(double x_meters, double y_meters, 
    sensor_data_t& data)
{
    data.landmarks_visible.clear();
    // Calculate range and bearing of each landmark. 
    // Landmarks must be arranged so that if they're w/in visual range then
    //  there's only one way to see them (i.e., not through a wall or on
    //  the back side of one).
    for (const landmark_description_t& landmark: g_landmarks)
    {
        double dx = x_meters - landmark.x_meters;
        double dy = y_meters - landmark.y_meters;
        double range_meters = sqrt(dx*dx + dy*dy);
        if (range_meters <= utils::LANDMARK_VISIBILITY_METERS)
        {
//printf("Close to landmark %.3f\n", range_meters);
            data.landmarks_visible.push_back(landmark);
        }
    }
}


void perform_sensor_sweep(double x_meters, double y_meters, sensor_data_t& data)
{
    // Perform 360-degree sensor sweep.
    calculate_ranges(x_meters, y_meters, data);
    calculate_landmarks(x_meters, y_meters, data);
}


// Build sensor view
////////////////////////////////////////////////////////////////////////


} // namespace slam_sim


#if defined(TEST_ANALYZE)

constexpr const char* WORLD_MAP_FILE = "../data/map.json";

using namespace slam_sim;
using namespace utils;

void print_sensor_data(sensor_data_t& data)
{
    double step_degs = 360.0 / (double) data.num_radials;
    for (uint32_t i=0; i<data.range_meters.size(); i++)
    {
        double range = data.range_meters[i];
        double theta = (double) i * step_degs;
        printf("%3d   %.3d   %6.2f\n", i, (int32_t) round(theta), range);
    }
    for (landmark_description_t& lm: data.landmarks_visible)
    {
        printf("Landmark %d (%s) at %.2f,%.2f\n", lm.id, lm.name.c_str(),
            lm.x_meters, lm.y_meters);
    }
}


static uint32_t check_radial(uint32_t num, double expected,
    const sensor_data_t& data)
{
    uint32_t errs = 0;
    if (fabs(data.range_meters[num] - expected) > 0.01)
    {
        fprintf(stderr, "Radial %d has range of %.3f, expected %.3f\n",
            num, data.range_meters[num], expected);
        errs++;
    }
    return errs;
}


int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    uint32_t errs = 0;
    ////////////////////////////////////////////////////////////////////
    load_world_map(WORLD_MAP_FILE);
    std::string response;
    sensor_data_t data;
    perform_sensor_sweep(4.0, 0.6, data);
    // wall distances
    errs += check_radial(0, 0.5, data);
    errs += check_radial(22, 0.7, data);
    errs += check_radial(45, -1.0, data);
    errs += check_radial(90, -1.0, data);
    errs += check_radial(134, 3.9, data);
    // table distances
    // 190 degs. Strikes vertical edge of table near bottom.
    //    sqrt(0.5^2 + (0.5/tan(10))^2)
    errs += check_radial(95, 2.879, data);
    // 196 degs. Strikes horizontal edge of table near right edge.
    //    sqrt(1.9^2 + (1.9*tan(16))^2)
    errs += check_radial(98, 1.977, data);
    // 216 degs. Strikes horizontal edge of table near left edge.
    //    sqrt(1.9^2 + (1.9*tan(36))^2)
    errs += check_radial(108, 2.349, data);
    // Check landmarks.
    if (data.landmarks_visible.size() != 1)
    {
        fprintf(stderr, "Expected seeing 1 landmark but saw %d\n",
            (int32_t) data.landmarks_visible.size());
        errs++;
    }
    else
    {
        if (data.landmarks_visible[0].id != 1)
        {
            fprintf(stderr, "Expected seeing landmark #1 but saw %d\n",
                data.landmarks_visible[0].id);
            errs++;
        }
    }
    ////////////////////////////////////////////////////////////////////
    if (errs > 0) {
        fprintf(stderr, "********************************\n");
        fprintf(stderr, "Encountered %d errors\n", errs);
    } else {
        fprintf(stderr, "All tests pass\n");
    }
    return (int32_t) errs;
}

#endif // TEST_ANALYZE
