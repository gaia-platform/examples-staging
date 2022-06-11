////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#pragma once

#include <string>

namespace actions
{

void stranger_detected();

void no_eligible_events(uint64_t person_id);

void base_credentials_required(uint64_t person_id);

void no_entry_right_now(uint64_t person_id, std::string room_name, std::string building_name);

void not_this_building(uint64_t person_id, std::string building_name);

void not_this_room(
    uint64_t person_id, std::string room_name, std::string building_name);

} // namespace actions
