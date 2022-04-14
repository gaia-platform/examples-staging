#pragma once

namespace slam_sim
{

// Degree/Radian conversions.
constexpr float c_rad_to_deg = 180.0 / M_PI;
constexpr float c_deg_to_rad = M_PI / 180.0;

// Width (height) of grids in occupancy grid.
constexpr float c_map_node_width_meters = 0.1;

// Max distance range sensors can get range data.
constexpr float c_range_sensor_max_meters = 4.0;
// Width of range sensor sweep in front of bot.
constexpr float c_range_sensor_sweep_degs = 90.0;
constexpr int32_t c_num_range_radials = 45;

} // namespace slam_sim

