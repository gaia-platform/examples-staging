////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Landmarks are uniqely identifiable locations in the environment.
// The description stores their position and an identifier.
//
////////////////////////////////////////////////////////////////////////

#include "landmark_description.hpp"

namespace utils
{

landmark_description_t::landmark_description_t()
{
    this->name = "undefined";
    this->id = -1;
    this->x_meters = 0.0;
    this->y_meters = 0.0;
}

landmark_description_t::landmark_description_t(std::string name, int32_t id,
    double x_meters, double y_meters)
{
    this->name = name;
    this->id = id;
    this->x_meters = x_meters;
    this->y_meters = y_meters;
}

} // namespace utils
