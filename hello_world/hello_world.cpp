#include "gaia/system.hpp"
#include <gaia/logger.hpp>

#include "gaia_hello_world.h"

int main()
{
    gaia::system::initialize();

    gaia_log::app().info("Hello World example is running...");

    gaia::db::begin_transaction();
    gaia::hello_world::entity_t::insert_row("World");
    // [Try uncommenting the following line and see how the program output changes]
    // gaia::hello_world::entity_t::insert_row("Alice");
    gaia::db::commit_transaction();

    gaia::system::shutdown();
}
