###################################################
# Copyright (c) Gaia Platform Authors
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
###################################################

cmake_minimum_required(VERSION 3.16)

project(clearpath)

set(CMAKE_CXX_STANDARD 17)

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include("/opt/gaia/cmake/gaia.cmake")

# --- Generate Direct Access classes from DDL---
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/gaia/clearpath.ddl
  DATABASE_NAME clearpath
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/gaia/clearpath.ruleset
  DATABASE_NAME clearpath
  CLANG_PARAMS
    -I ${PROJECT_SOURCE_DIR}/include
)

#
# Direct Access Example
#
add_executable(clearpath_direct_access
  src/main_direct_access.cpp
  src/graph.cpp
)

target_add_gaia_generated_sources(clearpath_direct_access)

target_include_directories(clearpath_direct_access PRIVATE
  ${GAIA_INC}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(clearpath_direct_access PRIVATE
  ${GAIA_LIB}
  Threads::Threads
)

#
# Rules Example
#
add_executable(clearpath_rules
  src/main_rules.cpp
  src/graph.cpp
)

target_add_gaia_generated_sources(clearpath_rules)

target_include_directories(clearpath_rules PRIVATE
  ${GAIA_INC}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(clearpath_rules PRIVATE
  ${GAIA_LIB}
  Threads::Threads
)
