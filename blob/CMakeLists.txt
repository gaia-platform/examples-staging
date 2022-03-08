#############################################
# Copyright (c) Gaia Platform LLC
# All rights reserved.
#############################################

cmake_minimum_required(VERSION 3.16)

enable_testing()

project(blob)

set(CMAKE_CXX_STANDARD 17)

set(GAIA_COMPILE_FLAGS "-c -Wall -Wextra -ggdb")
set(GAIA_LINK_FLAGS "-ggdb")

add_executable(test_blob
  test_blob.cpp
  blob.cpp
)

set_target_properties(test_blob PROPERTIES COMPILE_FLAGS "${GAIA_COMPILE_FLAGS}")
set_target_properties(test_blob PROPERTIES LINK_FLAGS "${GAIA_LINK_FLAGS}")
target_include_directories(test_blob PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(test_blob PRIVATE rt)

add_test(NAME test_blob COMMAND test_blob)
