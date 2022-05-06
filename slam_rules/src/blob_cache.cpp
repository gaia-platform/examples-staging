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
    RETAIL_ASSERT(
        size > 0,
        "Attempted to allocate a blob of 0 size!");

    m_id = id;
    m_size = size;
    m_id_superseded_blob = id_superseded_blob;
    m_data = new uint8_t[size]();
}


blob_t::~blob_t()
{
    RETAIL_ASSERT(
        m_data != nullptr,
        "Deleting blob NULL data element. Was it double deleted?");

    delete[] m_data;
    m_data = nullptr;
}

////////////////////////////////////////////////////////////////////////

blob_t* blob_cache_t::create_blob(uint32_t id, size_t size, 
    uint32_t id_superseded_blob)
{
    if ((size == 0) || (id == c_invalid_blob_id))
    {
        return nullptr;
    }

    std::unique_lock unique_lock(m_lock);

    // Lookup superseded blob. If it exists, and if it itself has a
    //  superseded blob, then delete the latter.
    if (id_superseded_blob != c_invalid_blob_id)
    {
        blob_t* superseded_blob = nullptr;
        superseded_blob = get_blob_(id_superseded_blob);
        if (superseded_blob != nullptr)
        {
            uint32_t corpse_id = superseded_blob->supersedes();
            if (corpse_id != c_invalid_blob_id) {
                // Delete corpse and remove all references to it.
                blob_t* corpse = get_blob_(corpse_id);
                delete corpse;
                m_blob_map[corpse_id] = nullptr;
                superseded_blob->m_id_superseded_blob = c_invalid_blob_id;
            }
            
        }
    }

    // Check if a blob with this id already exists. If so, reuse it. If not,
    //  create a new one and insert it into our map.
    blob_t* blob = get_blob_(id);
    if (blob == nullptr)
    {
        blob = new blob_t(id, size, id_superseded_blob);
        m_blob_map.insert(std::pair<uint32_t, blob_t*>(id, blob));
    }
    return blob;
}


blob_t* blob_cache_t::get_blob(uint32_t id)
{
    if (id == c_invalid_blob_id)
    {
        return nullptr;
    }
    std::shared_lock shared_lock(m_lock);
    return get_blob_(id);
}


blob_t* blob_cache_t::get_blob_(uint32_t id)
{
    RETAIL_ASSERT(
        id != c_invalid_blob_id,
        "Attempting to retrieve a blob using an invalid id!");

    return (m_blob_map.count(id) > 0) ? m_blob_map[id] : nullptr;
}

} // namespace slam_sim

