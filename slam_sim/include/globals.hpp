////////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////////////////////////

#pragma once

#include "blob_cache.hpp"

namespace slam_sim
{

// Flag to indicate app should exit. Normally this is 0. When it's time
//  to quit it's set to 1.
extern int32_t g_quit;

// Blob caches for area maps and working maps.
extern blob_cache_t g_area_blobs;
extern blob_cache_t g_working_blobs;

} // namespace slam_sim

