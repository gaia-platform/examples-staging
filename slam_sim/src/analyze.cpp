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
#include "slam_sim.hpp"

namespace slam_sim
{

using std::vector;
using std::string;
using nlohmann::json;
using utils::line_segment_t;

using gaia::direct_access::auto_transaction_t;
using gaia::slam::graph_t;
using gaia::slam::graph_writer;
using gaia::slam::observations_t;
using gaia::slam::lidar_data_t;

static observations_t g_observation;   // most recent observation
static graph_t g_graph;         // present graph

static double g_x_pos = 0.0;
static double g_y_pos = 0.0;
static double g_heading_degs = 0.0;

static double g_x_est = 0.0;
static double g_y_est = 0.0;
static double g_heading_est_degs = 0.0;

static double g_dx = 0.0;
static double g_dy = 0.0;
static double g_dheading_degs = 0.0;

static double g_dx_est = 0.0;
static double g_dy_est = 0.0;
static double g_dheading_est_degs = 0.0;

constexpr int32_t NUM_RAYS = 30;
constexpr int32_t FOV_DEGS = 60.0;

// Line segments describing outline of all objects in the world.
vector<line_segment_t> g_world_lines;
// Raw json of world map.
char g_world_map[JSON_BUFFER_LEN];

static std::vector<float> g_radial;
static std::vector<float> g_range;
static string g_sensor_data;

static std::mutex g_map_mutex;


// Set and parse json map data. Assume that the format is correct.
void set_map(const char* map)
{
    g_map_mutex.lock();
    // Make a copy of supplied map json (unless pointer is to actual
    //  json map).
    if (map != g_world_map) {
        size_t len = strlen(map);
        assert(len < sizeof g_world_map);
        strcpy(g_world_map, map);
    }
    // copy map data into line segment array
    const json& world_map = json::parse(g_world_map);
    const json& objects = world_map["world"];
    for (uint32_t i=0; i<objects.size(); i++) {
        const json& vertices = objects[i]["vertices"];;
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
    g_map_mutex.unlock();
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
    assert(len < JSON_BUFFER_LEN);
    if (fread(g_world_map, len, 1, fp) != 1)
    {
        fprintf(stderr, "Failed to read in world map\n");
        exit(1);
    }
    fclose(fp);
    set_map(g_world_map);
}

void get_map(std::string& response)
{
    response = g_world_map;
}

// Load map
////////////////////////////////////////////////////////////////////////
// Build sensor view

constexpr char SENSOR_DATA_POSITION[] = 
    "\"position\": { \"x\": %.3f, \"y\": %.3f, \"heading_deg\": %.3f }";

constexpr char SENSOR_MOTION[] = 
    "\"motion\": { \"dx\": %.3f, \"dy\": %.3f, \"head_deg\": %.3f }";

constexpr char SENSOR_DATA_EST_POSITION[] = 
    "\"est_position\": { \"x\": %.1f, \"y\": %.1f, \"heading_deg\": %.1f }";

constexpr char SENSOR_EST_MOTION[] = 
    "\"est_motion\": { \"dx\": %.1f, \"dy\": %.1f, \"heading_deg\": %.1f }";

constexpr char SENSOR_LIDAR[] = 
    "{ \"bearing\": %.1f, \"range\": %.1f }";

static void build_sensor_view(std::string& response)
{
    char buf[16384];
    int idx = 0;
    idx += sprintf(&buf[idx], "{ \n");
    idx += sprintf(&buf[idx], SENSOR_DATA_POSITION, g_x_pos, g_y_pos,
            g_heading_degs);
    idx += sprintf(&buf[idx], ", \n");
    idx += sprintf(&buf[idx], SENSOR_MOTION, g_dx, g_dy, g_dheading_degs);
    idx += sprintf(&buf[idx], ", \n");
    idx += sprintf(&buf[idx], "\"senses\": { \n");
    idx += sprintf(&buf[idx], SENSOR_DATA_EST_POSITION, g_x_est, g_y_est,
            g_heading_est_degs);
    idx += sprintf(&buf[idx], ", \n");
    idx += sprintf(&buf[idx], SENSOR_EST_MOTION, g_dx_est, g_dy_est, g_dheading_est_degs);
    idx += sprintf(&buf[idx], ", \n");
    idx += sprintf(&buf[idx], "\"lidar\": [ \n");
    for (uint32_t i=0; i<g_radial.size(); i++) 
    {
        idx += sprintf(&buf[idx], SENSOR_LIDAR, g_radial[i], g_range[i]);
        if (i < NUM_RAYS-1) 
        {
            idx += sprintf(&buf[idx], ", \n");
        }
    }
    idx += sprintf(&buf[idx], "\n] \n} \n}\n\n");
    g_sensor_data = buf;
    response = buf;
}


void calculate_sensors(std::string& response)
{
    g_range.clear();
    g_radial.clear();
    for (uint32_t n=0; n<NUM_RAYS; n++)
    {
        double theta = g_heading_degs + -FOV_DEGS/2.0 
                + (double) n * FOV_DEGS / (double) (NUM_RAYS-1);
        double min_dist = -1.0;
        for (uint32_t i=0; i<g_world_lines.size(); i++)
        {
            line_segment_t& seg = g_world_lines[i];
            double dist = seg.intersect_range(g_x_pos, g_y_pos, theta);
            if (dist > 0.0) {
                if ((min_dist < 0.0) || (dist < min_dist)) {
                    min_dist = dist;
                }
            }
        }
        g_range.push_back(min_dist);
        g_radial.push_back(theta);
    }
    build_sensor_view(response);
    g_dx = 0;
    g_dy = 0;
    g_dheading_degs = 0.0;
    g_dx_est = 0;
    g_dy_est = 0;
    g_dheading_est_degs = 0.0;
    ////////////////////////////////////////////
    // Sensor input is calculated. Create an observation point.
    auto_transaction_t tx;
    int32_t observation_num = 0;
    if (g_graph.num_observations() > 0) {
        // Existing graph. Get the next observation number.
        observation_num = g_observation.num() + 1;
    }
    observations_t observation = 
        observations_t::get(observations_t::insert_row(
            observation_num,    // num
            g_graph.id(),       // graph_id
            0.0, 0.0, 0.0,      // measured dx, dy and ddegs
            0.0, 0.0, 0.0, 1.0, // estimated dx, dy, ddegs, confidence
            0.0, 0.0, 0.0, 1.0  // calculated x, y, heading_degs, confidence
            ));
    if (g_graph.num_observations() == 0) {
        // New graph. Link it to this observation.
        observation.parent().connect(g_graph);
        g_graph.first_observation().connect(observation);
    } else {
        // This is a subsequent observation on an existing graph.
        //  Link the observations.
        observation.next().connect(g_observation);
        g_observation.prev().connect(observation);
    }
    // Update present observation.
    g_observation = observation;
    // Update number of observations in graph.
    graph_writer w = g_graph.writer();
    w.num_observations = g_graph.num_observations() + 1;
    w.update_row();
    // Build lidar data.
    lidar_data_t view = lidar_data_t::get(lidar_data_t::insert_row(
            g_radial,     // theta_deg
            g_range       // distance
            ));
    view.observation().connect(g_observation);
    g_observation.lidar().connect(view);
    tx.commit();
}

// Build sensor view
////////////////////////////////////////////////////////////////////////
// Position

// There's an analytical way to estimate the path a vehicle will
//  follow when its wheels are turned a certain amount, as a function
//  of wheel base. We're going for an easier solution here. The
//  input turn_degs_per_unit is how many degrees the robot turns
//  when it moves forward one unit of distance (e.g., one pixel).
//  To simplify things further, movement is approximated by
//  advancing very small amounts at a time.
void move_robot(double distance, double turn_degs_per_unit)
{
    double step_size = 0.05;   // step size to advance, in pixels
    double tot_dist = 0.0;
    double d_theta = turn_degs_per_unit * step_size;
    while (tot_dist < distance) {
        double dx = step_size * sin(utils::D2R * g_heading_degs);
        double dy = -step_size * cos(utils::D2R * g_heading_degs);
        tot_dist += step_size;
        // change in motion
        g_dx += dx;
        g_dy += dy;
        g_dheading_degs += d_theta;
        // estimated change in motion
        g_dx_est += dx;
        g_dy_est += dy;
        g_dheading_est_degs += d_theta;
        // actual movement
        g_x_pos += dx;
        g_y_pos += dy;
        g_heading_degs += d_theta;
        // estimated movement
        g_x_est += dx;
        g_y_est += dy;
        g_heading_est_degs += d_theta;
    }
    g_heading_degs = utils::unwrap_compass(g_heading_degs);
    g_heading_est_degs = utils::unwrap_compass(g_heading_est_degs);
}


void set_position(double x_pos, double y_pos, double heading_degs)
{
    g_x_pos = x_pos;
    g_y_pos = y_pos;
    g_heading_degs = heading_degs;
    //
    g_x_est = x_pos;
    g_y_est = y_pos;
    g_heading_est_degs = heading_degs;
    //
    g_dx = 0.0;
    g_dy = 0.0;
    g_dheading_degs = 0.0;
    //
    g_dx_est = 0.0;
    g_dy_est = 0.0;
    g_dheading_est_degs = 0.0;
    // When a new position is set this can create a discontinuity or
    //  can represent movement to a ground truth, among other
    //  posibilities. Either way, create a new graph to track motion
    //  going forward from here.
    auto_transaction_t tx;
    int32_t graph_id = g_graph.id();
    g_graph = graph_t::get(graph_t::insert_row(
        graph_id + 1,   // id
        x_pos,          // start_x
        y_pos,          // start_y
        heading_degs,   // start_heading_degs
        0               // num_observations
        ));
    tx.commit();

}

// Position
////////////////////////////////////////////////////////////////////////


} // namespace slam_sim

#if defined(TEST_ANALYZE)

using namespace slam_sim;

int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    load_default_map();
    set_position(280.0, 280.0, 315.0);
    std::string response;
    calculate_sensors(response);
    std::cout << response << std::endl;
    move_robot(30.0, 1.0);
    calculate_sensors(response);
    std::cout << response << std::endl;
    move_robot(5.0, 1.0);
    move_robot(5.0, 1.0);
    move_robot(5.0, 1.0);
    calculate_sensors(response);
    std::cout << response << std::endl;
    return 0;
}

#endif // TEST_ANALYZE

