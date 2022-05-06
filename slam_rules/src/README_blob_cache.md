# A mechanism for storing non-persistent memory blobs.

Must support blob allocation during a transaction and be resistent 
to memory leaks when transactions are rolled back and retried.

Primary use case is creating a memory allocation that stores non-primary 
data that's used during other computations. Non-primary means that the 
data in the blob can be (re)generated anytime from content stored in the 
database.

If a blob exists it can be used by the calling process.
If it doesn't exist then the calling process is responsible for 
initializing it.

One need for such a blob is for storing map data in SLAM, as navigation 
requires a large blob that represents obstacles and routes to a destination.

The database stores an ID for the blob, and sends that ID to the blob 
factory, asking for the memory associated w/ that blob. If the blob doesn't 
exist it is created. The factory needs to communicate to the calling process 
whether the blob existed or if it was freshly created.
This is done by using get() to fetch the blob. If it exists a valid pointer
to it is returned. Otherwise NULL is returned, in which case the calling
processes needs to call create(). If create() is called and
the blob with a given ID already exists, then the existing blob is returned.
The calling function will not necessarily know if it's a new or existing blob,
although it's guaranteed that a new blob will have its data field initialized
to NULL (e.g., uint8\_t\* data = new uint8\_t\[size]]();)


If blobs are created and destroyed, there must be a safe way to delete old 
ones, and this must operate in a world where a delete call may be issued 
many times (e.g., if called repeatedly during retried transactions) and 
should be tolerant of a delete being ignored (e.g., failed transaction 
that's not retried).

This is achieved by each blob having a reference to a blob that it 
supersedes, so if A is allocated and B is meant to supersede A (e.g., 
because of a resize), then a pointer to A is stored within B.
Even though A is superseded, it remains a valid memory allocation until 
B is superseded.

Only when B is superseded will A be deleted.

If B is allocated but the transaction is rolled back and not retried, 
then A will remain in use. B will be allocated but won't be used until 
the calling process asks the blob factory for a blob w/ B's ID.

As noted earlier, the calling process must manage the IDs for memory.
A simple way to do this is to store the blob ID in a table. When creating a new blob during a transaction, increment the ID and ask the factory for a blob with this new ID.
If the transaction is rolled back, the newly created blob will have been allocated but the ID will not be
incremented, so if it's retried it will request a blob with the same ID again.
That request can simultaneously have the former ID be superseded.
E.g.,:
```
  blob_t* blob = cache.create_blob(
      ID+1,   // new blob ID
      size_in_bytes,  // size of memory to be allocated
      ID      // superceded blob
  );
```
