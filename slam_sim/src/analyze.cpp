#include <iostream>
#include <string>
#include <mutex>
#include <vector>

#include "gaia/system.hpp"
#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>
#include <gaia/exceptions.hpp>

#include "slam/gaia_slam.h"

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
using utils::landmark_description_t;
using utils::sensor_data_t;

constexpr int32_t NUM_RANGE_RADIALS = 180;

// Line segments describing outline of all objects in the world.
vector<line_segment_t> g_world_lines;
vector<landmark_description_t> g_landmarks;


// for main.cpp or high-level utils
////////////////////////////////////////////////////////////////////////


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


void load_default_map()
{
    FILE* fp = fopen(DEFAULT_WORLD_MAP_PATH, "r");
    if (!fp)
    {
        fprintf(stderr, "Failed to open %s\n", DEFAULT_WORLD_MAP_PATH);
        exit(1);
    }
    fseek(fp, 0, SEEK_END);
    size_t len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char* buf = new char[len+1]; 
    if (fread(buf, len, 1, fp) != 1)
    {
        fprintf(stderr, "Failed to read world map %s\n", 
            DEFAULT_WORLD_MAP_PATH);
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
        if (range_meters <= LANDMARK_VISIBILITY_METERS)
        {
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



//    build_sensor_view(response);
//    g_dx = 0;
//    g_dy = 0;
//    g_dheading_degs = 0.0;
//    g_dx_est = 0;
//    g_dy_est = 0;
//    g_dheading_est_degs = 0.0;
//    ////////////////////////////////////////////
//    // Sensor input is calculated. Create an observation point.
//    auto_transaction_t tx;
//    int32_t observation_num = 0;
//    if (g_graph.num_observations() > 0) {
//        // Existing graph. Get the next observation number.
//        observation_num = g_observation.num() + 1;
//    }
//    observations_t observation = 
//        observations_t::get(observations_t::insert_row(
//            observation_num,    // num
//            g_graph.id(),       // graph_id
//            0.0, 0.0, 0.0,      // measured dx, dy and ddegs
//            0.0, 0.0, 0.0, 1.0, // estimated dx, dy, ddegs, confidence
//            0.0, 0.0, 0.0, 1.0  // calculated x, y, heading_degs, confidence
//            ));
//    if (g_graph.num_observations() == 0) {
//        // New graph. Link it to this observation.
//        observation.parent().connect(g_graph);
//        g_graph.first_observation().connect(observation);
//    } else {
//        // This is a subsequent observation on an existing graph.
//        //  Link the observations.
//        observation.next().connect(g_observation);
//        g_observation.prev().connect(observation);
//    }
//    // Update present observation.
//    g_observation = observation;
//    // Update number of observations in graph.
//    graph_writer w = g_graph.writer();
//    w.num_observations = g_graph.num_observations() + 1;
//    w.update_row();
//    // Build lidar data.
//    lidar_data_t view = lidar_data_t::get(lidar_data_t::insert_row(
//            g_radial,     // theta_deg
//            g_range       // distance
//            ));
//    view.observation().connect(g_observation);
//    g_observation.lidar().connect(view);
//    tx.commit();
//}

// Build sensor view
////////////////////////////////////////////////////////////////////////


//void set_position(double x_pos, double y_pos)
//{
//    g_x_pos_meters = x_pos;
//    g_y_pos_meters = y_pos;
//    g_heading_degs = heading_degs;
//    //
//    g_x_est = x_pos;
//    g_y_est = y_pos;
//    g_heading_est_degs = heading_degs;
//    //
//    g_dx = 0.0;
//    g_dy = 0.0;
//    g_dheading_degs = 0.0;
//    //
//    g_dx_est = 0.0;
//    g_dy_est = 0.0;
//    g_dheading_est_degs = 0.0;
//    // When a new position is set this can create a discontinuity or
//    //  can represent movement to a ground truth, among other
//    //  posibilities. Either way, create a new graph to track motion
//    //  going forward from here.
//    auto_transaction_t tx;
//    int32_t graph_id = g_graph.id();
//    g_graph = graph_t::get(graph_t::insert_row(
//        graph_id + 1,   // id
//        x_pos,          // start_x
//        y_pos,          // start_y
//        heading_degs,   // start_heading_degs
//        0               // num_observations
//        ));
//    tx.commit();
//
//}

// Position
////////////////////////////////////////////////////////////////////////


} // namespace slam_sim

#if defined(TEST_ANALYZE)

using namespace slam_sim;
using namespace utils;

void print_sensor_data(sensor_data_t& data)
{
    for (uint32_t i=0; i<data.radials.length(); i++)
    {
        int32_t n = data.landmark_num[i];
        double range = data.range_meters[i];
        double theta = data.range_meters[i];
        printf("%3d   %.3d   %6.2f    %s\n", i, (int32_t) round(theta),
            range, n<0 ? "" : "*");
    }
}


int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    load_default_map();
    std::string response;
    sensor_data_t data;
    calculate_sensors(4.0, 0.6, data);
    print_sensor_data(data);

//    
//    std::cout << response << std::endl;
//    move_robot(30.0, 1.0);
//    calculate_sensors(response);
//    std::cout << response << std::endl;
//    move_robot(5.0, 1.0);
//    move_robot(5.0, 1.0);
//    move_robot(5.0, 1.0);
//    calculate_sensors(response);
//    std::cout << response << std::endl;
    return 0;
}

#endif // TEST_ANALYZE
