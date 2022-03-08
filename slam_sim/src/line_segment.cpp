#include <iostream>
#include <cmath>

#include "line_segment.hpp"

using namespace std;

namespace utils
{

line_segment_t::line_segment_t(double x0, double y0, double x1, double y1)
{
    m_x0 = x0;
    m_y0 = y0;
    m_x1 = x1;
    m_y1 = y1;
    // derived values
    // Y value is in screen coords (+Y is downward). Invert that for
    //  computations.
    m_a = y0 - y1;
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
    double y1 = -m_y0;
    double y2 = -m_y1;
    // Segment 2
    double x3 = x;
    double x4 = x3 + 10.0 * sin(D2R * theta);
    double y3 = -y;
    double y4 = -y + 10.0 * cos(D2R * theta);
//printf("This segment:   %.1f,%.1f -> %.1f,%.1f\n", x1, y1, x2, y2);
//printf(" -> line %.1f,%.1f at %.1f\n", x3, y3, theta);
//printf("       other:   %.1f,%.1f -> %.4f,%.4f\n", x3, y3, x4, y4);
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
    if (fabs(denom) > 1.0e-6) {
        double px = (xy12 * x3_x4 - xy34 * x1_x2) / denom;
        double py = (xy12 * y3_y4 - xy34 * y1_y2) / denom;
//printf("  intersect point %.4f,%.4f\n", px, py);
        // Intersection may or may not be in the direction of theta.
        //  All this algorithm tells us is that the lines, infinitely
        //  expanded, meet at this point.
        double inter_theta = unwrap_compass(R2D * atan2(px-x3, py-y3));
        double delta_degs = inter_theta - theta;
        if (delta_degs < -180) {
            delta_degs += 360.0;
        } else if (delta_degs > 1800) {
            delta_degs -= 360.0;
        }
//printf("    measured theta: %.1f   delta: %.1f\n", inter_theta, delta_degs);
        if (fabs(delta_degs) < 1.0)
        {
            // intersection point is in direction of theta
            double dist0 = measure_distance(px-x1, py-y1);
            double dist1 = measure_distance(px-x2, py-y2);
            if ((dist0 <= m_len) && (dist1 <= m_len))
            {
                dist = measure_distance(px-x3, py-y3);
            }
        }
    }   // Otherwise lines are parallel.
//printf("    range %.1f\n", dist);
    return dist;
}


} // namespace utils

#if defined(TEST_LINE_SEGMENT)

using utils::line_segment_t;

uint32_t check_distance(double len, double expected)
{
    if (fabs(len - expected) > 0.001) {
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
    // 
    if (errs > 0) {
        fprintf(stderr, "********************************\n");
        fprintf(stderr, "Encountered %d errors\n", errs);
    } else {
        fprintf(stderr, "All tests pass\n");
    }

    return 0;
}

#endif // TEST_LINE_SEGMENT

