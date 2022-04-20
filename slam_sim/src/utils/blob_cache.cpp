////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include "blob_cache.hpp"
#include "retail_assert.hpp"

namespace slam_sim
{

blob_t::blob_t(uint32_t id, size_t size, uint32_t id_superseded_blob)
{
    reset(id, size, id_superseded_blob);
}

blob_t::~blob_t()
{
    clear();
}

void blob_t::clear()
{
    if (data != nullptr)
    {
        delete[] data;
        data = nullptr;
    }
}

void blob_t::reset(uint32_t id, size_t size, uint32_t id_superseded_blob)
{
    RETAIL_ASSERT(
        size > 0,
        "Attempted to allocate a blob of 0 size!");

    clear();

    this->id = id;
    this->size = size;
    this->id_superseded_blob = id_superseded_blob;

    this->state = blob_state_t::initialized;

    data = new uint8_t[size];
}


blob_t* blob_cache_t::create_or_reset_blob(uint32_t id, size_t size, 
    uint32_t id_superseded_blob)
{
    RETAIL_ASSERT(
        id != c_invalid_blob_id,
        "Attempting to create a blob with an invalid id!");
    RETAIL_ASSERT(
        size > 0,
        "Attempting to create a blob with a 0 size!");

    std::unique_lock unique_lock(m_lock);

    // Lookup superseded blob.
    blob_t* superseded_blob_ptr = nullptr;
    if (id_superseded_blob != c_invalid_blob_id)
    {
        superseded_blob_ptr = get_blob_locked(id_superseded_blob);
    }

    // Check if a blob with this id already exists;
    // if it exists, then reuse it;
    // if not, then create a new one and insert it in our map.
    bool is_new_blob = false;
    blob_t* blob_ptr = get_blob_locked(id);
    if (blob_ptr)
    {
        blob_ptr->reset(id, size, id_superseded_blob);
    }
    else
    {
        is_new_blob = true;
        blob_ptr = new blob_t(id, size, id_superseded_blob);
        m_blob_map.insert(std::pair<uint32_t, blob_t*>(id, blob_ptr));
    }

    // Check if we have a previously superseded blob to delete.
    // We only need to do this for new blobs, because old blobs have 
    //  already gone through this.
    if ((is_new_blob && superseded_blob_ptr) && 
        (superseded_blob_ptr->id_superseded_blob != c_invalid_blob_id))
    {
        auto iterator = 
            m_blob_map.find(superseded_blob_ptr->id_superseded_blob);
        RETAIL_ASSERT(
            iterator != m_blob_map.end(),
            "Failed to find the previously superseded blob!");
        delete iterator->second;
        m_blob_map.erase(iterator);
    }
    return blob_ptr;
}

blob_t* blob_cache_t::get_blob(uint32_t id)
{
    RETAIL_ASSERT(
        id != c_invalid_blob_id,
        "Attempting to retrieve a blob using an invalid id!");

    std::shared_lock shared_lock(m_lock);

    auto iterator = m_blob_map.find(id);
    return (iterator == m_blob_map.end()) ? nullptr : iterator->second;
}

blob_t* blob_cache_t::get_blob_locked(uint32_t id)
{
    RETAIL_ASSERT(
        id != c_invalid_blob_id,
        "Attempting to retrieve a blob using an invalid id!");

    auto iterator = m_blob_map.find(id);
    return (iterator == m_blob_map.end()) ? nullptr : iterator->second;
}

} // namespace slam_sim

