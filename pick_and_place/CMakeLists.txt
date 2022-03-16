###################################################
# Copyright (c) Gaia Platform LLC
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
###################################################

cmake_minimum_required(VERSION 3.16)

# Set the project name.
project(pick_and_place)

set(CMAKE_CXX_STANDARD 17)

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include("/opt/gaia/cmake/gaia.cmake")

# --- Generate EDC from DDL---
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/gaia/pick_and_place.ddl
  DATABASE_NAME pick_and_place
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/gaia/pick_and_place.ruleset
)

add_executable(pick_and_place
  src/main.cpp
)

target_add_gaia_generated_sources(pick_and_place)

target_include_directories(pick_and_place
  PRIVATE
    ${GAIA_INC}
)
target_link_directories(pick_and_place PRIVATE ${GAIA_LIB})
target_link_libraries(pick_and_place PRIVATE ${GAIA_LIB} Threads::Threads)
