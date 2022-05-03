#pragma once
#include <math.h>
#include <stdint.h>

namespace slam_sim
{

// Degree/Radian conversions.
constexpr float c_rad_to_deg = (float) (180.0 / M_PI);
constexpr float c_deg_to_rad = (float) (M_PI / 180.0);

// Max distance range sensors can get range data.
constexpr float c_range_sensor_max_meters = 4.0f;
// Width of range sensor sweep in front of bot.
constexpr float c_range_sensor_sweep_degs = 90.0f;
constexpr int32_t c_num_range_radials = 45;

// Width (height) of grids in occupancy grid.
// Map for navigation
constexpr float c_navigation_map_node_width_meters = 0.25f;
// Map for output
constexpr float c_export_map_node_width_meters = 0.05f;

// Path finding.
// Bias factor for seeking out new areas to explore. This is the increase
//  in cost to traverse a node for each time it's been occupied.
constexpr float c_path_penalty_per_occupation = 0.5f;

// The Dijkstra-like path finding algorithms are very rough on the local
//  scale, having at most 8 directions to move away from a node in the map.
//  The approach here is to smooth the path direction by looking ahead
//  X nodes and using this direction necessar to go to that node
//  to generate the direction vector. The constant here is that X.
constexpr uint32_t c_num_ancestors_for_direction = 5;

// Distance travelled between each simulated step.
constexpr float c_step_meters = 0.8 * c_navigation_map_node_width_meters;

// How close to destination bot has to be to declare victory.
constexpr float c_destination_radius_meters = 0.5f;

} // namespace slam_sim

