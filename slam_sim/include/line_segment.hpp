////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Line segment representation. Assumptions here are that coordinate
//  space uses screen coordinates, such that increasing Y is downward,
//  while theta is compass-based, with 0 degrees being north, and upward.
//
////////////////////////////////////////////////////////////////////////

#pragma once

namespace utils
{

constexpr double R2D = 180.0 / M_PI;
constexpr double D2R = M_PI / 180.0;

class line_segment_t
{
public:
    line_segment_t(double x0, double y0, double x1, double y1);

    double length()  { return m_len;   }

    // Distance from x,y to the point of intersection on this line
    //  segment along the path theta_deg. If no intersection (e.g.,
    //  theta is moving away from segment, or if it's parallel)
    //  then a negative number is returned.
    // theta_deg is compass-based (0 is north, 90 is east, etc)
    double intersect_range(double x, double y, double theta_deg);

//protected:
    double m_x0, m_x1;
    double m_y0, m_y1;
    // derived values
    double m_a;  // A = y1 - y0 = dy
    double m_b;  // B = x1 - x0 = dx
    double m_c;  // C = A*x0 + B*y0
    double m_len;
};

// Remaps degrees to be on [0,360)
double unwrap_compass(double theta_degs);

} // namespace utils
