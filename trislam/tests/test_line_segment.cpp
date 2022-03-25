////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>

#include "line_segment.hpp"

using namespace std;

using utils::line_segment_t;

uint32_t check_distance(double len, double expected)
{
    if (fabs(len - expected) > 0.001)
    {
        fprintf(stderr, "Distance out of range. Got %.3f, expected %.3f\n",
            len, expected);
        return 1;
    }
    return 0;
}

int main()
{
    uint32_t errs = 0;

    // horizontal line at y=-10 (i.e., 10 units above the origin)
    line_segment_t a(-10.0, -10.0, 10.0, -10.0);
    errs += check_distance(a.intersect_range(0.0, 0.0, 0.0), 10.0);
    errs += check_distance(a.intersect_range(0.0, 0.0, 30.0), 11.547);
    errs += check_distance(a.intersect_range(0.0, 0.0, -30.0), 11.547);
    errs += check_distance(a.intersect_range(0.0, 0.0, 60.0), -1.0);
    errs += check_distance(a.intersect_range(0.0, 0.0, 150.0), -1.0);
    errs += check_distance(a.intersect_range(0.0, 0.0, 270.0), -1.0);

    // diagonal line crossing from 0,-10 to 10,0 (i.e., 1st quadrant)
    line_segment_t b(0.0, -10.0, 10.0, 0.0);
    errs += check_distance(b.intersect_range(0.0, 0.0, 0.1), 9.983);
    errs += check_distance(b.intersect_range(0.0, 0.0, -0.01), -1.0);
    errs += check_distance(b.intersect_range(0.0, 0.0, 359.99), -1.0);
    errs += check_distance(b.intersect_range(0.0, 0.0, 45.0), 7.071);
    errs += check_distance(b.intersect_range(0.0, 0.0, 225.0), -1.0);

    // opposite orientation horizontal line at y=10
    line_segment_t c(10.0, -10.0, -10.0, -10.0);
    errs += check_distance(c.intersect_range(0.0, 0.0, 0.0), 10.0);
    errs += check_distance(c.intersect_range(0.0, 0.0, 30.0), 11.547);
    errs += check_distance(c.intersect_range(0.0, 0.0, -30.0), 11.547);
    errs += check_distance(c.intersect_range(0.0, 0.0, 60.0), -1.0);
    errs += check_distance(c.intersect_range(0.0, 0.0, 150.0), -1.0);
    errs += check_distance(c.intersect_range(0.0, 0.0, 270.0), -1.0);

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
