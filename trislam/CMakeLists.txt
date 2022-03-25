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

# Default compiler/linker flags.
add_compile_options(-c -Wall -Wextra -ggdb)

# --- Generate Direct Access classes from DDL---
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/gaia/slam.ddl
  DATABASE_NAME slam
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/gaia/slam.ruleset
  DATABASE_NAME slam
  CLANG_PARAMS
    -I ${PROJECT_SOURCE_DIR}/include
)

# Application

add_executable(slam_sim
  src/analyze.cpp
  src/globals.cpp
  src/navigation.cpp
  src/occupancy.cpp
  src/paths.cpp
  src/slam_sim.cpp
  src/utils/coord_list.cpp
  src/utils/line_segment.cpp
)

target_add_gaia_generated_sources(slam_sim)

target_include_directories(slam_sim PRIVATE
  ${GAIA_INC}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_directories(slam_sim PRIVATE ${GAIA_LIB_DIR})
target_link_libraries(slam_sim PRIVATE ${GAIA_LIB} rt Threads::Threads)


########################################################################
# Tests

################################
add_executable(test_coord_list
  tests/test_coord_list.cpp
  src/utils/coord_list.cpp
)

set_target_properties(test_coord_list PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_coord_list PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_coord_list PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_coord_list PRIVATE rt)

add_test(NAME test_coord_list COMMAND test_coord_list)


################################
add_executable(test_line_segment
  tests/test_line_segment.cpp
  src/utils/line_segment.cpp
)

set_target_properties(test_line_segment PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_line_segment PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_line_segment PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_line_segment PRIVATE rt)

add_test(NAME test_line_segment COMMAND test_line_segment)


################################
add_executable(test_analyze
  tests/test_analyze.cpp
  src/analyze.cpp
  src/globals.cpp
  src/navigation.cpp
  src/occupancy.cpp
  src/paths.cpp
  src/utils/line_segment.cpp
  src/utils/landmark_description.cpp
)
target_add_gaia_generated_sources(test_analyze)

set_target_properties(test_analyze PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_analyze PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_analyze PRIVATE ${GAIA_INC})
target_include_directories(test_analyze PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_analyze PRIVATE ${GAIA_LIB} rt)

add_test(NAME test_analyze COMMAND test_analyze)
