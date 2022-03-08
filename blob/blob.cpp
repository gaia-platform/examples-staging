/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "blob.hpp"
#include "retail_assert.hpp"

std::map<uint32_t, blob_t*> blob_t::s_blob_map;

blob_t* blob_t::create_blob(uint32_t id, size_t size, uint32_t id_superseded_blob)
{
    // Lookup superseded blob.
    blob_t* superseded_blob_ptr = nullptr;
    if (id_superseded_blob != c_invalid_blob_id)
    {
        superseded_blob_ptr = get_blob(id_superseded_blob);
    }

    // Create new blob.
    blob_t* new_blob_ptr = new blob_t(id, size, id_superseded_blob);
    s_blob_map.insert(std::pair<uint32_t, blob_t*>(id, new_blob_ptr));

    // Check if we have a previously superseded blob to delete.
    if (superseded_blob_ptr && superseded_blob_ptr->m_id_superseded_blob != c_invalid_blob_id)
    {
        auto iterator = s_blob_map.find(superseded_blob_ptr->m_id_superseded_blob);

        RETAIL_ASSERT(
            iterator != s_blob_map.end(),
            "Failed to find the previously superseded blob!");

        s_blob_map.erase(iterator);
    }

    return new_blob_ptr;
}

blob_t* blob_t::get_blob(uint32_t id)
{
    RETAIL_ASSERT(
        id != c_invalid_blob_id,
        "Attempting to retrieve a blob using an invalid id!");

    auto iterator = s_blob_map.find(id);
    return (iterator == s_blob_map.end()) ? nullptr : iterator->second;
}

blob_t::blob_t(uint32_t id, size_t size, uint32_t id_superseded_blob)
    : m_id(id), m_size(size), m_id_superseded_blob(id_superseded_blob)
{
    m_data = new uint8_t[size];
}

blob_t::~blob_t()
{
    if (m_data != nullptr)
    {
        delete[] m_data;
        m_data = nullptr;
    }
}
