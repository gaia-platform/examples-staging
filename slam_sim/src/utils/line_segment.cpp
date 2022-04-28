////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// The simulator relies on "ray tracing" in a 2D world, with walls
//  being represented as line segments. If a radial from the bot
//  intersects a line segment (wall) then the intersection of that
//  radial with the wall determines the range of an object on that
//  radial. The nearest line segment on each radial is selected which
//  will correspond to the nearest wall.
//
// Coordinate system is map-based, with positive Y being upward and 
//  positive X being rightward. It is NOT the coordinate system for
//  image processing (which has positive Y being downward).
//
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>

#include "line_segment.hpp"

using namespace std;

namespace slam_sim
{


line_segment_t::line_segment_t(double x0, double y0, double x1, double y1)
{
    m_x0 = x0;
    m_y0 = y0;
    m_x1 = x1;
    m_y1 = y1;
    // derived values
    m_a = y1 - y0;
    m_b = x1 - x0;
    m_c = m_a*x0 + m_b*y0;
    m_len = sqrt(m_a*m_a + m_b*m_b);
}

static double measure_distance(double dx, double dy)
{
    return sqrt(dx*dx + dy*dy);
}


// returns value on [0,360)
double unwrap_compass(double theta)
{
    double degs = theta;
    while (degs < 0.0)
    {
        degs += 360.0;
    }
    while (degs >= 360.0)
    {
        degs -= 360.0;
    }
    return degs;
}


double line_segment_t::intersect_range(double x, double y, double theta_deg)
{
    // move theta to [0,360)
    double theta = unwrap_compass(theta_deg);
    // Determine intersect point px,py
    // Algorithm from wikipedia.
    //    https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    // Segment 1
    double x1 = m_x0;
    double x2 = m_x1;
    double y1 = m_y0;
    double y2 = m_y1;
    // Segment 2
    double x3 = x;
    double x4 = x3 + 10.0 * sin(D2R * theta);
    double y3 = y;
    double y4 = y + 10.0 * cos(D2R * theta);
//printf("This segment:   %.1f,%.1f -> %.1f,%.1f\n", x1, y1, x2, y2);
//printf(" -> line %.1f,%.1f at %.1f\n", x3, y3, theta);
//printf("    becomes:   %.1f,%.1f -> %.4f,%.4f\n", x3, y3, x4, y4);
    // Px = ((x1y2 - x2y1)(x3-x4) - (x1-x2)(x3y4-y3x4)) / denom;
    // Py = ((x1y2 - y1x2)(y3-y4) - (y1-y2)(x3y4-y3x4)) / denom;
    // denom = (x1-x2)(y3-y4) - (y1-y2)(x3-x4);
    // breaking that down into smaller elements:
    double x3_x4 = x3 - x4;
    double x1_x2 = x1 - x2;
    double y3_y4 = y3 - y4;
    double y1_y2 = y1 - y2;
    double xy12 = x1*y2 - x2*y1;
    double xy34 = x3*y4 - y3*x4;
    double denom = x1_x2*y3_y4 - y1_y2*x3_x4;
    double dist = -1.0;
    // Before computing px,py, make sure denominator is non-zero.
    if (fabs(denom) > 1.0e-6)
    {
        double px = (xy12 * x3_x4 - xy34 * x1_x2) / denom;
        double py = (xy12 * y3_y4 - xy34 * y1_y2) / denom;
//printf("  intersect point %.4f,%.4f\n", px, py);
        // Intersection may or may not be in the direction of theta.
        //  All this algorithm tells us is that the lines, infinitely
        //  expanded, meet at this point. Find out if intersection
        //  point is in the direction of theta.
        double inter_theta = unwrap_compass(R2D * atan2(px-x3, py-y3));
        double delta_degs = inter_theta - theta;
        if (delta_degs < -180.0)
        {
            delta_degs += 360.0;
        }
        else if (delta_degs > 180.0)
        {
            delta_degs -= 360.0;
        }
//printf("    measured theta: %.1f   delta: %f\n", inter_theta, delta_degs);
        if (fabs(delta_degs) < 1.0)
        {
            // Intersection point is in direction of theta. Now see if
            //  it falls w/in the bounds of this line segment, or if the
            //  point of intersection is on an extension of this line.
            // Distance from each end line segment to the intersection point:
            double dist0 = measure_distance(px-x1, py-y1);
            double dist1 = measure_distance(px-x2, py-y2);
            // If longest distance is less than line segment length then
            //  the intersection point is w/in the segment.
//printf("dist 0: %f\n", dist0);
//printf("dist 1: %f\n", dist1);
            if ((dist0 <= m_len) && (dist1 <= m_len))
            {
                dist = measure_distance(px-x3, py-y3);
            }
        }
    }   // Otherwise lines are parallel.
//printf("    range %.1f\n", dist);
    return dist;
}


} // namespace slam_sim

