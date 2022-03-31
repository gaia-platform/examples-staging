////////////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <gaia/logger.hpp>

#include "gaia/system.hpp"

#include "gaia_hello_world.h"

int main() {

  gaia::system::initialize();

  gaia_log::app().info("Hello example is running...");

  gaia::db::begin_transaction();
  gaia::hello_world::names_t::insert_row("Alice");
  gaia::db::commit_transaction();

  gaia::system::shutdown();
}
