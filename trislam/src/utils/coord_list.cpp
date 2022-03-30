#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include "coord_list.hpp"

using std::string;

namespace slam_sim
{

const double c_rad_to_deg = 180.0 / M_PI; 

coord_list_t::coord_list_t(string& file_name)
{
    std::ifstream in_file(file_name);
    if (!in_file)
    {
        fprintf(stderr, "Failed to load %s: %s\n", file_name.c_str(),
            strerror(errno));
        exit(1);
    }
    string str;
    bool first_round = true;
    double x_prev, y_prev;
    double x, y;
    try
    {
        while(getline(in_file, str))
        {
            if (str.size() < 2)
            {
                continue;
            }
            if (str[0] == '#')
            {
                continue;
            }
            std::istringstream iss(str);
            iss >> x;
            iss >> y;
            if (first_round)
            {
                first_round = false;
            }
            else
            {
                double dx = x - x_prev;
                double dy = y - y_prev;
                double theta = c_rad_to_deg * atan2(dx, dy);
                if (theta < 0.0)
                {
                    theta += 360.0;
                }
                map_coord_t coord;
                coord.x_meters = (x + x_prev) / 2.0;
                coord.y_meters = (y + y_prev) / 2.0;
                coord.heading_degs = theta;
                push_back(coord);
            }
            x_prev = x;
            y_prev = y;
        }
    }
    catch (std::exception e)
    {
        std::cout << "Error prarsing " << file_name << ": " << e.what() << "\n";
    }
    in_file.close();
}

} // namespace slam_sim 

