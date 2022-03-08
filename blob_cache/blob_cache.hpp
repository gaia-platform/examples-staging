/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include <cstddef>
#include <cstdint>

#include <unordered_map>
#include <shared_mutex>

// A mechanism for storing non-persistent memory blobs.
// ​
// Must support blob allocation during a transaction and be resistent
// to memory leaks when transactions are rolled back and retried.
// ​
// Primary use case is creating a memory allocation that stores non-primary
// data that's used during other computations. Non-primary means that the
// data in the blob can be (re)generated anytime from content stored in the
// database. If a blob exists it can be used by the calling process. If it
// doesn't exist then the calling process is responsible for initializing
// it.
// ​
// One need for such a blob is for storing map data in SLAM, as navigation
// requires a large blob that represents obstacles and routes to a destination.
// ​
// The database stores an ID for the blob, and sends that ID to the blob
// factory, asking for the memory associated w/ that blob. If the blob
// doesn't exist it is created. The factory needs to communicate to the
// calling process whether the blob existed or if it was freshly created.
// One way to do this is to have a flag in the blob that indicates if its
// data is valid/current.
// ​
// The ID is managed by the calling process and that's controlled by the
// ID. If blobs are created and destroyed, there must be a safe way to
// delete old ones, and this must operate in a world where a delete call
// may be issued many times (e.g., if called repeatedly during retried
// transactions) and should be tolerant of a delete being ignored (e.g.,
// failed transaction that's not retried). This is achieved by each blob
// having a reference to a blob that it supersedes, so if A is allocated
// and B is meant to supersede A (e.g., because of a resize), then a pointer
// to A is stored within B. Even though A is superseded, it remains a
// valid memory allocation until B is superseded. Only when B is superseded
// will A be deleted. If B is allocated but the transaction is rolled back
// and not retried, then A will remain in use. B will be allocated but
// won't be used until the calling process asks the blob factory for a
// blob w/ B's ID.
// ​
// As noted earlier, the calling process must manage the IDs for memory.
// A simple way to do this is to store the blob ID in a table. When creating
// a new blob during a transaction, increment the ID and ask the factory
// for a blob with this new ID. If the transaction is rolled back, the
// newly created blob will have been allocated but the ID will not be
// incremented, so if it's retried it will request a blob with the same ID
// again. That request can simultaneously have the former ID be
// superseded. E.g.,:
//   blob_t* blob = blob_factory_t.create_blob(
//       ID+1,   // new blob ID
//       size_in_bytes,  // size of memory to be allocated
//       ID      // superceded blob
//   );

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
