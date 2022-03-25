#include "coor_loader.hpp"

constexpr const char* g_test_file = "../data/test_coords.txt";

using slam_sim::map_coord_t;

uint32_t check_coord(map_coord_t& coord, double x, double y, double theta)
{
    uint32_t errs = 0;
    if (fabs(x - coord.x_meters) > 0.001) {
        fprintf(stderr, "X coordinate error. Expected %f, got %f\n", x,
            coord.x_meters);
    }
    if (fabs(y - coord.y_meters) > 0.001) {
        fprintf(stderr, "Y coordinate error. Expected %f, got %f\n", y,
            coord.y_meters);
    }
    if (fabs(theta - coord.heading_degs) > 0.001) {
        fprintf(stderr, "Heading error. Expected %f, got %f\n", theta,
            coord.heading_degs);
    }
    return 0;
}

uint32_t test_path_file(string test_file)
{
    uint32_t errs = 0;
    printf("Testing read and parse of coordinate file '%s'\n", 
        test_file.c_str());
    ////////////////////////////////////////////////////////////////////
    slam_sim::coord_list_t coords(test_file);
    errs += check_coord(coords[0], 1.0, 1.5, 0.0);
    errs += check_coord(coords[1], 0.0, 2.0, 270.0);
    errs += check_coord(coords[2], -3.0, 0.0, 225.0);
    errs += check_coord(coords[3], 0.0, -2.0, 90.0);

    ////////////////////////////////////////////////////////////////////
    return errs;
}

int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    uint32_t errs = 0;
    ////////////////////////////////////////////////////////////////////
    errs += test_path_file(g_test_file);

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

