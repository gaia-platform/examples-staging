///////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//
// Transaction wrapper. Transactions cannot presently be nested. This
//  means that each function accessing the database must be aware of
//  where it's being called from, and whether there's an existing
//  transaction open or not. To eliminate the boilerplate in each
//  function, a transaction wrapper is used. Begin and commit calls
//  are made to the wrapper, and the wrapper keeps track of whether
//  an existing transaction is open or not. If so, the begin/commit
//  are ignored as those will be part of the outer transaction.
//
// There is no action on destruction -- it is entirely up to the user
//  to call 'commit()'. Likewise, it is up to the user to catch any
//  transaction errors.
//
///////////////////////////////////////////////////////////////////////

#pragma once

namespace slam_sim
{

class txn_t
{
public:
    txn_t();
    ~txn_t() { ; }

    // Returns true if command was actually executed (i.e., if an existing
    //  transaction was not already open).
    bool begin();
    bool commit();

protected:
    //bool m_existing_transaction;
    bool m_manage_txn;
};

} // namespace slam_sim

