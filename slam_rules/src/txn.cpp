///////////////////////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//
// Transaction wrapper. See txn.hpp for description.
//
///////////////////////////////////////////////////////////////////////

#include <gaia/db/db.hpp>

#include "txn.hpp"

namespace slam_sim
{

txn_t::txn_t()
{
    m_manage_txn = !gaia::db::is_transaction_open();
}

bool txn_t::begin()
{
    if (m_manage_txn)
    {
        gaia::db::begin_transaction();
    }
    return m_manage_txn;
}

bool txn_t::commit()
{
    if (m_manage_txn)
    {
        gaia::db::commit_transaction();
    }
    return m_manage_txn;
}

} // namespace slam_sim

