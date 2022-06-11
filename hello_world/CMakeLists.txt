###################################################
# Copyright (c) Gaia Platform Authors
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
###################################################

cmake_minimum_required(VERSION 3.16)

project(hello_world)

set(CMAKE_CXX_STANDARD 17)

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include("/opt/gaia/cmake/gaia.cmake")

# --- Generate Direct Access classes from DDL---
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/hello_world.ddl
  DATABASE_NAME hello_world
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/hello_world.ruleset
)

add_executable(hello_world
  hello_world.cpp
)

target_add_gaia_generated_sources(hello_world)
target_include_directories(hello_world PRIVATE ${GAIA_INC})
target_link_directories(hello_world PRIVATE ${GAIA_LIB_DIR})
target_link_libraries(hello_world PRIVATE gaia rt Threads::Threads)
