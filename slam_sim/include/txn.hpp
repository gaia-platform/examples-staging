///////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//
// Transaction wrapper. Some API code that accesses the database
//  may need to be called from both within rules and from main(). As
//  there are no nested transactions, a transaction is open when calling
//  from within a rule, and it's not automatically open when called
//  procedurally, a smart wrapper for transactions is needed in the API
//  that can begin/commit only if a transaction isn't already open.
//
// It is of course possible to require that code calling the API be
//  aware of what the API functions require, but that can result in
//  convoluted code, and it also works against code compartmentalization.
//
// This simple wrapper detects if a transaction is already open. If so,
//  a user's call to 'begin()' will be ignored, as will any subsequent
//  call to 'commit()'.
//
// There is no action on destruction -- it is entirely up to the user to
//  call 'commit()'. Likewise, it is up to the user to catch any
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

