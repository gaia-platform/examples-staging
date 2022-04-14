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

#include "constants.hpp"

namespace slam_sim
{


class line_segment_t
{
public:
    line_segment_t(float x0, float y0, float x1, float y1);

    float length()  { return m_len;   }

    // Distance from x,y to the point of intersection on this line
    //  segment along the path theta_deg. If no intersection (e.g.,
    //  theta is moving away from segment, or if it's parallel)
    //  then a negative number is returned.
    // theta_deg is compass-based (0 is north, 90 is east, etc)
    float intersect_range(float x, float y, float theta_deg);

//protected:
    float m_x0, m_x1;
    float m_y0, m_y1;
    // derived values
    float m_a;  // A = y1 - y0 = dy
    float m_b;  // B = x1 - x0 = dx
    float m_c;  // C = A*x0 + B*y0
    float m_len;
};

// Remaps degrees to be on [0,360)
float unwrap_compass(float theta_degs);

} // namespace slam_sim
