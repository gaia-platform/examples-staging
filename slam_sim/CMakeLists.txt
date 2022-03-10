###################################################
# Copyright (c) Gaia Platform LLC
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
###################################################

cmake_minimum_required(VERSION 3.16)

enable_testing()

project(slam_sim)

set(CMAKE_CXX_STANDARD 17)

set(GAIA_COMPILE_FLAGS "-c -Wall -Wextra -ggdb")
set(GAIA_LINK_FLAGS "-ggdb")

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include("/opt/gaia/cmake/gaia.cmake")

# --- Generate Direct Access classes from DDL---
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/gaia/slam.ddl
  DATABASE_NAME slam
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/gaia/slam.ruleset
  DATABASE_NAME slam
    -I ${PROJECT_SOURCE_DIR}/include
)

#
# Application
#
# add_executable(slam_sim
#   src/blob_cache.cpp
#   src/line_segment.cpp
#   src/mapping.cpp
# )

# target_add_gaia_generated_sources(slam_sim)

# target_include_directories(slam_sim PRIVATE
#   ${GAIA_INC}
#   ${PROJECT_SOURCE_DIR}/include
# )

# target_link_libraries(slam_sim PRIVATE
#   ${GAIA_LIB}
#   Threads::Threads
# )

add_executable(test_blob_cache
  tests/test_blob_cache.cpp
  src/blob_cache.cpp
)

set_target_properties(test_blob_cache PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_blob_cache PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_blob_cache PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_blob_cache PRIVATE rt)

add_test(NAME test_blob_cache COMMAND test_blob_cache)
