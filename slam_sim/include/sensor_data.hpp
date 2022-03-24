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

#include "landmark_description.hpp"

namespace utils
{

// How close Alice has to be to see the landmark.
constexpr double LANDMARK_VISIBILITY_METERS = 3.0;

// Max range of range sensor.
constexpr double RANGE_SENSOR_MAX_METERS = 4.0;

struct sensor_data_t
{
    int32_t num_radials;

    // Range on each radial. Range is -1 if there's no data.
    std::vector<float> range_meters;

    // 0 or more landmarks are visible
    std::vector<landmark_description_t> landmarks_visible;
};

} // namespace utils
