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

void test_blob_cache();

int main(void)
{
    test_blob_cache();
}

void test_blob_cache()
{
    // Create two blobs.
    blob_cache_t::get()->create_or_reset_blob(1, 1, 0);
    blob_cache_t::get()->create_or_reset_blob(2, 1, 1);

    // Read back the information of the blobs that we created.
    blob_t* blob_ptr = blob_cache_t::get()->get_blob(1);
    cout << "Blob 1 has superseded blob with id '" << blob_ptr->id_superseded_blob << "'" << endl;

    blob_ptr = blob_cache_t::get()->get_blob(2);
    cout << "Blob 2 has superseded blob with id '" << blob_ptr->id_superseded_blob << "'" << endl;

    // Create a third blob. It should lead to the deletion of the first blob.
    blob_cache_t::get()->create_or_reset_blob(3, 1, 2);

    blob_ptr = blob_cache_t::get()->get_blob(3);
    cout << "Blob 3 has superseded blob with id '" << blob_ptr->id_superseded_blob << "'" << endl;

    // Also update this blob's state.
    blob_ptr->state = blob_state_t::used;
    cout << "Blob 3 state is now '" << (size_t)blob_ptr->state << "'" << endl;

    // Verify that the first blob has been deleted.
    blob_ptr = blob_cache_t::get()->get_blob(1);
    RETAIL_ASSERT(
        !blob_ptr, "FAILED: Blob 1 still exists!");
    cout << "Blob 1 has been removed, as expected!" << endl;

    // Reset the third blob state.
    blob_cache_t::get()->create_or_reset_blob(3, 1, 2);

    // Read the third blob and verify that its state has been reset.
    blob_ptr = blob_cache_t::get()->get_blob(3);
    RETAIL_ASSERT(
        blob_ptr->state == blob_state_t::initialized,
        "FAILED: Blob 3 state has not been reset!");
    cout << "Blob 3 has been reset, as expected!" << endl;
}
