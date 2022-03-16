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

include(FetchContent)

# Checkout lifthttp library (to make HTTP requests).
FetchContent_Declare(
  lifthttp
  GIT_REPOSITORY https://github.com/jbaldwin/liblifthttp.git
  GIT_TAG        v2021.2
)
FetchContent_MakeAvailable(lifthttp)

# Clang-8 (which gaiat is based on) doesn't like noexcept with default.
# This is a workaround to remove the "noexcept" keyword and make gaiat happy.
# Reference: https://stackoverflow.com/a/29483269/1214125
file(READ ${lifthttp_SOURCE_DIR}/inc/lift/response.hpp response_data)
string(REPLACE "noexcept" "" response_data "${response_data}")
file(WRITE ${lifthttp_SOURCE_DIR}/inc/lift/response.hpp "${response_data}")

file(READ ${lifthttp_SOURCE_DIR}/inc/lift/request.hpp request_data)
string(REPLACE "noexcept" "" request_data "${request_data}")
file(WRITE ${lifthttp_SOURCE_DIR}/inc/lift/request.hpp "${request_data}")

# Checkout JSON library.
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)
FetchContent_MakeAvailable(json)

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
  CLANG_PARAMS
    -I ${lifthttp_SOURCE_DIR}/inc
    -I ${json_SOURCE_DIR}/include
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
target_link_libraries(mqtt PRIVATE ${GAIA_LIB} AWS::aws-crt-cpp lifthttp nlohmann_json::nlohmann_json Threads::Threads)
