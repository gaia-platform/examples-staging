////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <chrono>

#include <thread>

#include <gaia/db/db.hpp>
#include <gaia/logger.hpp>
#include <gaia/system.hpp>

using std::this_thread::sleep_for;

constexpr uint32_t c_rule_wait_millis = 100;

/**
 * Wait an arbitrary amount of time for rule execution to terminate.
 * Rules are triggered after commit and can take some time to fully execute.
 */
void wait_for_rules()
{
    sleep_for(std::chrono::milliseconds(c_rule_wait_millis));
}

template <typename T_type>
void clear_table()
{
    for (auto obj_it = T_type::list().begin(); obj_it != T_type::list().end(); )
    {
        auto current_obj_it = obj_it++;
        current_obj_it->delete_row();
    }
}

void clear_data()
{
}

int main()
{
    gaia::system::initialize();

    // We explicitly handle the transactions with begin_transaction() and commit_transaction()
    // to trigger the rules.
    gaia::db::begin_transaction();
    clear_data();
    gaia::db::commit_transaction();

    gaia_log::app().info("=== Creates a new Graph and observe the corresponding rules triggered ===");

    wait_for_rules();

    gaia::system::shutdown();
}
