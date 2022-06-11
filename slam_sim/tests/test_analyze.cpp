////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
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
#include "sensor_data.hpp"
#include "slam_sim.hpp"

constexpr const char* WORLD_MAP_FILE = "../data/map.json";

using namespace slam_sim;
using namespace utils;

void print_sensor_data(sensor_data_t& data)
{
    double step_degs = 360.0 / (double) data.num_radials;
    for (uint32_t i=0; i<data.range_meters.size(); i++)
    {
        double range = data.range_meters[i];
        double theta = (double) i * step_degs;
        printf("%3d   %.3d   %6.2f\n", i, (int32_t) round(theta), range);
    }
    for (landmark_description_t& lm: data.landmarks_visible)
    {
        printf("Landmark %d (%s) at %.2f,%.2f\n", lm.id, lm.name.c_str(),
            lm.x_meters, lm.y_meters);
    }
}


static uint32_t check_radial(uint32_t num, double expected,
    const sensor_data_t& data)
{
    uint32_t errs = 0;
    if (fabs(data.range_meters[num] - expected) > 0.01)
    {
        fprintf(stderr, "Radial %d (%d degs) has range of %.3f, "
            "expected %.3f\n", num, 2*num, data.range_meters[num], expected);
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
    perform_sensor_sweep(-3.0, -4.4, data);

    // wall distances
    errs += check_radial(0, 0.5, data);
    errs += check_radial(22, 0.7, data);
    errs += check_radial(45, -1.0, data);
    errs += check_radial(90, -1.0, data);
    errs += check_radial(134, 3.9, data);

    // table distances
    // 190 degs. Strikes vertical edge of table near bottom.
    //    sqrt(0.5^2 + (0.5/tan(10))^2)
    errs += check_radial(95, 2.879, data);

    // 196 degs. Strikes horizontal edge of table near right edge.
    //    sqrt(1.9^2 + (1.9*tan(16))^2)
    errs += check_radial(98, 1.977, data);

    // 216 degs. Strikes horizontal edge of table near left edge.
    //    sqrt(1.9^2 + (1.9*tan(36))^2)
    errs += check_radial(108, 2.349, data);

    // Check landmarks.
    if (data.landmarks_visible.size() != 1)
    {
        fprintf(stderr, "Expected seeing 1 landmark but saw %d\n",
            (int32_t) data.landmarks_visible.size());
        errs++;
    }
    else
    {
        if (data.landmarks_visible[0].id != 1)
        {
            fprintf(stderr, "Expected seeing landmark #1 but saw %d\n",
                data.landmarks_visible[0].id);
            errs++;
        }
    }

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
