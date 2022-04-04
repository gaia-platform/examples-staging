////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Sensor data of the bot's environment.
//
////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>

namespace slam_sim
{

struct map_coord_t
{
    float x_meters;
    float y_meters;
    float heading_degs;
};

// Max range of range sensor.
constexpr double RANGE_SENSOR_MAX_METERS = 4.0;

struct sensor_data_t
{
    uint32_t num_radials;

    // Range on each radial. Range is -1 if there's no data.
    std::vector<float> range_meters;

    // Radials for each measured range.
    std::vector<float> bearing_degs;
};

} // namespace slam_sim
