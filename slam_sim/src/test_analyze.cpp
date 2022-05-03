////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <mutex>
#include <vector>

#include <gaia/system.hpp>

#include "json.hpp"
#include "line_segment.hpp"
#include "map_types.hpp"
#include "sensor_data.hpp"
#include "slam_sim.hpp"

constexpr const char* WORLD_MAP_FILE = "../data/map.json";

using namespace slam_sim;
using namespace slam_sim;

void print_sensor_data(sensor_data_t& data)
{
    for (uint32_t i=0; i<data.range_meters.size(); i++)
    {
        double range = data.range_meters[i];
        double theta = data.bearing_degs[i];
        printf("%3d   %.3d   %6.2f\n", i, (int32_t) round(theta), range);
    }
}


static uint32_t check_radial(uint32_t num, double expected,
    const sensor_data_t& data)
{
    uint32_t errs = 0;
    if (fabs(data.range_meters[num] - expected) > 0.01)
    {
        fprintf(stderr, "Radial %d (%.1f degs) has range of %.3f, "
            "expected %.3f\n", num, data.bearing_degs[num],
            data.range_meters[num], expected);
        errs++;
    }
    return errs;
}


int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    uint32_t errs = 0;

    gaia::system::initialize();

    ////////////////////////////////////////////////////////////////////
    load_world_map(WORLD_MAP_FILE);
    std::string response;
    sensor_data_t data;
    map_coord_t coord = {
        .x_meters = -3.0,
        .y_meters = 4.4,
        .heading_degs = 45.0
    };
    calculate_range_data(coord, data);
    errs += check_radial(0, 0.5, data);
    errs += check_radial(22, 0.7, data);
    errs += check_radial(44, 2.9, data);

    coord.heading_degs = 135.0;
    calculate_range_data(coord, data);
    errs += check_radial(22, -1.0, data);

    // table distances
    // 207 degs. Strikes vertical edge of table near bottom.
    //    sqrt(1^2 + (1/tan(28))^2)
    coord.heading_degs = 208.0;
    calculate_range_data(coord, data);
    errs += check_radial(22, 2.130, data);

    // 228.5 degs. Strikes horizontal edge of table near right edge.
    //    sqrt(0.9^2 + (0.9/tan(41.5))^2)
    errs += check_radial(32, 1.358, data);

    // 245 degs. Strikes horizontal edge of table near left edge.
    //    sqrt(0.9^2 + (0.9*tan(25.2))^2)
    calculate_range_data(coord, data);
    errs += check_radial(40, 2.114, data);

    ////////////////////////////////////////////////////////////////////
    if (errs > 0)
    {
        fprintf(stderr, "********************************\n");
        fprintf(stderr, "Encountered %d errors\n", errs);
    }
    else
    {
        fprintf(stderr, "All tests pass\n");
    }

    return static_cast<int>(errs);
}
