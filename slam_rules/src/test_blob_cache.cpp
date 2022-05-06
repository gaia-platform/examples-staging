////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <cstring>

#include <iostream>

#include "blob_cache.hpp"
#include "retail_assert.hpp"

using namespace std;

namespace slam_sim
{

static int32_t test_blob_cache()
{
    uint32_t errs = 0;
    blob_cache_t cache;
    // Create blob.
    blob_t* blob1 = cache.create_blob(1, 16, 0);
    if (blob1 == nullptr)
    {
        fprintf(stderr, "Failed to create blob 1\n");
        errs++;
    }
    if (cache.get_blob(1) == nullptr)
    {
        fprintf(stderr, "Failed to fetch blob 1\n");
        errs++;
    }

    // Create blob 2, superseding 1.
    blob_t* blob2 = cache.create_blob(2, 16, 1);
    if (blob2 == nullptr)
    {
        fprintf(stderr, "Failed to create blob 2\n");
        errs++;
    }
    if (cache.get_blob(2) == nullptr)
    {
        fprintf(stderr, "Failed to fetch blob 2\n");
        errs++;
    }

    // Make sure 1's still around
    if (cache.get_blob(1) == nullptr)
    {
        fprintf(stderr, "Blob 1 delete prematurely\n");
        errs++;
    }

    // Create blob 3, superseding 2, which should eliminate 1
    blob_t* blob3 = cache.create_blob(3, 16, 2);
    if (blob3 == nullptr)
    {
        fprintf(stderr, "Failed to create blob 3\n");
        errs++;
    }
    if (cache.get_blob(3) == nullptr)
    {
        fprintf(stderr, "Failed to fetch blob 3\n");
        errs++;
    }
    // Make sure 2's still around
    if (cache.get_blob(2) == nullptr)
    {
        fprintf(stderr, "Blob 2 delete prematurely\n");
        errs++;
    }
    // Make sure 1 is gone
    if (cache.get_blob(1) != nullptr)
    {
        fprintf(stderr, "Blob 1 was not deleted\n");
        errs++;
    }

    // Try to create invalid blob.
    blob_t* blob0 = cache.create_blob(0, 16, 0);
    if (blob0 != nullptr)
    {
        fprintf(stderr, "Succeeded in creating invalid blob\n");
        errs++;
    }

    ////////////////////////////////////////////////////////////////////
    return errs;
}

} // namespace slam_sim


int main(void)
{
    uint32_t errs = 0;
    errs += slam_sim::test_blob_cache();
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

