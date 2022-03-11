#pragma once
#include <vector>

#include "landmark_description.hpp"

namespace utils
{

struct sensor_data_t
{
    int32_t num_radials;

    // Range on each radial. Range is -1 if there's no data.
    std::vector<float> range_meters;

    // 0 or more landmarks are visible
    std::vector<landmark_description_t> landmarks_visible;
};

} // namespace utils
