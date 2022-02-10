#pragma once
#include "gaia_slam.h"

namespace slam_sim
{

constexpr int32_t SATELLITE_PORT = 4501;
constexpr int32_t SIM_PORT = 4500;

constexpr int32_t PACKET_HEADER_LEN = 16;

constexpr char DEFAULT_WORLD_MAP_PATH[] = "map.json";

constexpr int32_t JSON_BUFFER_LEN = 16384;

// Set robot positio.n
void set_position(double x_pos, double y_pos, double heading_degs);

// Move robot with specified turn rate (essentially, degrees of turn per
//  pixel traveled).
void move_robot(double distance, double turn_degs_per_unit);

// Generates and returns present sensor view.
void calculate_sensors(std::string& response);

void get_map(std::string& response);

////////////////////////////////////////////////

void load_default_map();
void thread_shutdown_callback(int sig);
void* comm_thread_main(void* unused);

extern uint32_t g_quit;

} // namespace slam_sim

