////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////
//
// Summary description of a landmark.
//
////////////////////////////////////////////////////////////////////////
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
