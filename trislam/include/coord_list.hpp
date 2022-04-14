#pragma once
#include <string>
#include <vector>

namespace slam_sim
{

struct map_coord_t
{
    float x_meters;
    float y_meters;
    float heading_degs;
};

// coord_list is a vector of type map_coord_t. Constructor initializes 
//  vector with coordinates stored in file.
// File format:
//    <x0>  <y0>
//    <x1>  <y1>
//    ...
//    <xn>  <yn>
// There are (n-1) coordinates pulled from the file. The coordinates are
//  at the midpoint of each adjacent pair, and the heading is from the
//  first pair to the second. Pair values are separated by white space.
class coord_list_t : public std::vector<map_coord_t>
{
public:
    coord_list_t(std::string& file_name);
    coord_list_t(const char* file_name);

private:
    void init(std::string& file_name);
};

} // namespace slam_sim

