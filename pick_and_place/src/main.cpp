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

int main()
{
    gaia::system::initialize();

    gaia::db::begin_transaction();
    state_writer state_w;
    state_w.pick = true;
    state_w.place = false;
    state_w.insert_row();
    gaia::db::commit_transaction();

    gaia::system::shutdown();

    return 0;
}
