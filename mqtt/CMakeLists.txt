###################################################
# Copyright (c) Gaia Platform LLC
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
###################################################

cmake_minimum_required(VERSION 3.16)

# Set the project name.
project(mqtt)

set(CMAKE_CXX_STANDARD 17)

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# Default compiler/linker flags.
add_compile_options(-c -Wall -Wextra)

# Debug build-specific options.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g3 -ggdb -fno-limit-debug-info -fno-omit-frame-pointer -fno-optimize-sibling-calls -ggnu-pubnames -gsplit-dwarf)
  if(SANITIZER STREQUAL "ASAN")
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
  endif()
else()
  add_compile_options(-O3)
endif()

include("/opt/gaia/cmake/gaia.cmake")
set(MQTT_DDL ${PROJECT_SOURCE_DIR}/gaia/mqtt.ddl)
set(MQTT_RULESET ${PROJECT_SOURCE_DIR}/gaia/mqtt.ruleset)

# --- Generate EDC from DDL---
process_schema(
  DDL_FILE ${MQTT_DDL}
  DATABASE_NAME mqtt
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${MQTT_RULESET}
  DATABASE_NAME mqtt
)

add_executable(mqtt
  src/main.cpp
  src/utils.cpp
)

find_package(aws-crt-cpp REQUIRED)

target_add_gaia_generated_sources(mqtt)
add_dependencies(mqtt translate_mqtt_ruleset)
target_include_directories(mqtt
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
  PRIVATE
    ${GAIA_INC}
)
target_link_directories(mqtt PRIVATE ${GAIA_LIB})
target_link_libraries(mqtt PRIVATE AWS::aws-crt-cpp gaia Threads::Threads)
