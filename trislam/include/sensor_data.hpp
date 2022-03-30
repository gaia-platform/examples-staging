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

struct sensor_data_t
{
    uint32_t num_radials;

    float center_degs;

    // Range on each radial. Range is -1 if there's no data.
    std::vector<float> range_meters;
    std::vector<float> bearing_degs;
};

} // namespace slam_sim
