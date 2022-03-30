////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#pragma once

#include <cstdint>

namespace gaia::gaia_slam::vertex_type
{

static constexpr uint8_t c_lidar_scan = 0;
static constexpr uint8_t c_image_keyframe = 1;
static constexpr uint8_t c_visual_landmark = 2;

} // namespace gaia::gaia_slam::vertex_type
