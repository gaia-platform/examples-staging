#pragma once

namespace slam_sim
{

// Degree/Radian conversions.
constexpr float c_rad_to_deg = 180.0 / M_PI;
constexpr float c_deg_to_rad = M_PI / 180.0;

// Width (height) of grids in occupancy grid.
// Area map is to provide coarse guidance for route planning.
constexpr float c_area_map_node_width_meters = 0.25;
// Working map is local and provides finer local details for collision
//  avoidance.
constexpr float c_working_map_node_width_meters = 0.1;
// Output map is what's exported for observation.
constexpr float c_output_map_node_width_meters = 0.1;

// Max distance range sensors can get range data.
constexpr float c_range_sensor_max_meters = 4.0;
// Width of range sensor sweep in front of bot.
constexpr float c_range_sensor_sweep_degs = 90.0;
constexpr int32_t c_num_range_radials = 45;


} // namespace slam_sim

