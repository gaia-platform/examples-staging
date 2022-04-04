#pragma once

namespace slam_sim
{

// Degree/Radian conversions.
constexpr float c_rad_to_deg = (float) (180.0 / M_PI);
constexpr float c_deg_to_rad = (float) (M_PI / 180.0);

// Width (height) of grids in occupancy grid.
constexpr float c_map_node_width_meters = 0.1f;

// Max distance range sensors can get range data.
constexpr float c_range_sensor_max_meters = 4.0f;
// Width of range sensor sweep in front of bot.
constexpr float c_range_sensor_sweep_degs = 90.0f;
constexpr int32_t c_num_range_radials = 45;

constexpr float c_area_map_node_width_meters = 0.25f;
constexpr float c_working_map_node_width_meters = 0.05f;
constexpr float c_standard_map_node_width_meters = 0.05f;

} // namespace slam_sim

