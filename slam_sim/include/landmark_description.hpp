#pragma once
#include <string>

namespace utils
{

struct landmark_description_t
{
    landmark_description_t();
    landmark_description_t(std::string name, int32_t id, 
        double x_meters, double y_meters);

    std::string name;
    int32_t id;
    double x_meters;
    double y_meters;
};

} // namespace utils

