/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

#include <map>

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
// transactions) and should be tolerate of a delete being ignored (e.g.,
// failed transaction that's not retried). This is achieved by each blob
// having a reference to a blob that it supercedes, so if A is allocated
// and B is meant to supercede A (e.g., because of a resize), then a pointer
// to A is stored within B. Even though A is superceded, it remains a
// valid memory allocation until B is superceded. Only when B is superceded
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
// superceded. E.g.,:
//   blob_t* blob = blob_factory_t.create_blob(
//       ID+1,   // new blob ID
//       size_in_bytes,  // size of memory to be allocated
//       ID      // superceded blob
//   );

class blob_t
{
public:
    static constexpr uint32_t c_invalid_blob_id = 0;

    // Creates a blob with the specified ID and characteristics
    static blob_t* create_blob(uint32_t id, size_t size, uint32_t id_superseded_blob = c_invalid_blob_id);

    // Retrieves a blob given an id.
    // Will return nullptr if the blob doesn't exist.
    static blob_t* get_blob(uint32_t id);

public:
    // states are probably NEWLY_CREATED, VALID, and DIRTY
    uint32_t get_state() { return m_state; }
    void set_state(uint32_t new_state) { m_state = new_state;  }

    uint8_t* data() { return m_data; }
    size_t size() { return m_size; }

protected:
    blob_t(uint32_t id, size_t size, uint32_t id_superseded_blob);
    ~blob_t();

protected:
    uint32_t m_id;
    size_t m_size;
    // arbitrary blob content -- m_data = malloc(m_size)
    uint8_t* m_data;

    uint32_t m_state;

    // Blob that this blob supersedes.
    // It will be deleted when this blob itself is superseded.
    uint32_t m_id_superseded_blob;

protected:
    // Map of allocated blobs.
    static std::map<uint32_t, blob_t*> s_blob_map;
};
