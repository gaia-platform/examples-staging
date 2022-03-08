#############################################
# Copyright (c) Gaia Platform LLC
# All rights reserved.
#############################################

cmake_minimum_required(VERSION 3.16)

enable_testing()

project(blob_cache)

set(CMAKE_CXX_STANDARD 17)

set(GAIA_COMPILE_FLAGS "-c -Wall -Wextra -ggdb")
set(GAIA_LINK_FLAGS "-ggdb")

add_executable(test_blob_cache
  test_blob_cache.cpp
  blob_cache.cpp
)

set_target_properties(test_blob_cache PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_blob_cache PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_blob_cache PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(test_blob_cache PRIVATE rt)

add_test(NAME test_blob_cache COMMAND test_blob_cache)
