///////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
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

constexpr uint32_t c_invalid_blob_id = 0;

enum class blob_state_t : uint8_t
{
    not_set = 0,

    initialized = 1,
    used = 2,
};

struct blob_t
{
public:
    blob_t(uint32_t id, size_t size, uint32_t id_superseded_blob);
    ~blob_t();

    // Deallocate blob memory.
    void clear();

    // Reset blob content.
    void reset(uint32_t id, size_t size, uint32_t id_superseded_blob);

public:
    uint32_t id;

    // Blob data information.
    size_t size;
    uint8_t* data;

    // Blob state.
    blob_state_t state;

    // Blob that this blob supersedes.
    // It will be deleted when this blob itself is superseded.
    uint32_t id_superseded_blob;
};

class blob_cache_t
{
public:
    static blob_cache_t* get();

public:
    // Creates a blob with the specified ID and characteristics
    blob_t* create_or_reset_blob(uint32_t id, size_t size, uint32_t id_superseded_blob = c_invalid_blob_id);

    // Retrieves a blob given an id.
    // Will return nullptr if the blob doesn't exist.
    blob_t* get_blob(uint32_t id);

protected:
    blob_cache_t() = default;

    blob_cache_t(const blob_cache_t&) = delete;
    blob_cache_t& operator=(const blob_cache_t&) = delete;

protected:
    // Map of allocated blobs.
    std::unordered_map<uint32_t, blob_t*> m_blob_map;
    std::shared_mutex m_lock;

protected:
    static blob_cache_t s_blob_cache;
};
