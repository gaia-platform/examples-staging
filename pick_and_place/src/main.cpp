////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include "gaia/system.hpp"

#include "gaia_pick_and_place.h"

using namespace gaia::pick_and_place;

/**
 * Remove all the objects for the given type T_type.
 */
template <typename T_type>
void clear_table()
{
    for (auto obj_it = T_type::list().begin(); obj_it != T_type::list().end();)
    {
        auto current_obj_it = obj_it++;
        current_obj_it->delete_row();
    }
}

/**
 * Remove all rows in the database.
 */
void clear_data()
{
    clear_table<state_t>();
    clear_table<move_action_t>();
    clear_table<pick_action_t>();
}

int main()
{
    gaia::system::initialize();

    // Start clean.
    gaia::db::begin_transaction();
    clear_data();
    gaia::db::commit_transaction();

    // Initial state, should not trigger any rule.
    gaia::db::begin_transaction();
    state_t state = state_t::get(state_t::insert_row(true, true));
    gaia::db::commit_transaction();

    // Update the state to [pick && ~place] which should trigger a rule.
    gaia::db::begin_transaction();
    state_writer state_w = state.writer();
    state_w.pick = true;
    state_w.place = false;
    state_w.update_row();
    gaia::db::commit_transaction();

    gaia::system::shutdown();

    return 0;
}
