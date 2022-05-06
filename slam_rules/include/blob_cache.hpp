///////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//
// Mechanism for memory 'blobs' to be accessed by the database.
//
// These blobs are on the heap. A reference number (ID) is stored in
//  a database table and that ID is used to fetch a blob. Blobs are
//  not persistent and their content must be regenerated from database
//  content if they are not available.
//
// Blob creation and destruction is designed to be resistant to
//  transaction rollbacks and retries. See the README.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <cstdint>

#include <unordered_map>
#include <shared_mutex>

namespace slam_sim
{

constexpr uint32_t c_invalid_blob_id = 0;


class blob_t
{
public:
    blob_t(uint32_t id, size_t size, uint32_t id_superseded_blob);
    ~blob_t();

    uint32_t id()   { return m_id;    }
    size_t size()   { return m_size;  }
    uint8_t* data() { return m_data;  }

    uint32_t supersedes() { return m_id_superseded_blob;    }

protected:
    friend class blob_cache_t;

    uint32_t m_id;

    // Blob data information.
    size_t m_size;
    uint8_t* m_data;

    // Blob that this blob supersedes.
    // It will be deleted when this blob itself is superseded.
    uint32_t m_id_superseded_blob;
};


class blob_cache_t
{
public:
    blob_cache_t() = default;
    blob_cache_t(const blob_cache_t&) = delete;
    blob_cache_t& operator=(const blob_cache_t&) = delete;

    // Creates a blob with the specified ID and characteristics
    blob_t* create_blob(uint32_t id, size_t size, 
        uint32_t id_superseded_blob = c_invalid_blob_id);

    // Retrieves a blob given an id.
    // Will return nullptr if the blob doesn't exist.
    blob_t* get_blob(uint32_t id);

protected:
    // Map of allocated blobs.
    std::unordered_map<uint32_t, blob_t*> m_blob_map;
    std::shared_mutex m_lock;

    // Nested get_blob() call from w/in create_or_reset_blob(), where a
    //  mutex lock has already been established.
    blob_t* get_blob_(uint32_t id);
};

} // namespace slam_sim

