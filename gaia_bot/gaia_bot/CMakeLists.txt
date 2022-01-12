# Copyright 2022 Gaia Platform, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(gaia_bot)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include the Gaia CMake file.
include("/opt/gaia/cmake/gaia.cmake")

# Preparing the Gaia shared library.
add_library(gaia SHARED IMPORTED GLOBAL)
set_target_properties(gaia PROPERTIES IMPORTED_LOCATION ${GAIA_LIB_DIR}/libgaia.so)

# Automatically find the dependencies listed in package.xml
# instead of manually using `find_package()` for each package.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# Generate C++ Direct Access Classes from the DDL schema.
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/src/gaia_bot.ddl
  DATABASE_NAME gaia_bot
  OUTPUT_FOLDER include/${PROJECT_NAME}
)

# Translate the Ruleset into C++.
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/src/gaia_bot.ruleset
  CLANG_PARAMS
    -isystem${sensor_msgs_INCLUDE_DIRS} -I${PROJECT_SOURCE_DIR}/include
)

# Add a library for the ROS2 gaia_bot Node.
# We added a library instead of an executable to create a Composable Node,
# which allows dynamically loading multiple Nodes into a shared process.
# https://docs.ros.org/en/galactic/Concepts/About-Composition.html
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  DIRECTORY src
)

add_dependencies(${PROJECT_NAME}_node
  generate_${PROJECT_NAME}_direct_access
  translate_${PROJECT_NAME}_ruleset
)

# Add Gaia-generated C++, Gaia includes, and Gaia libs to library.
target_add_gaia_generated_sources(${PROJECT_NAME}_node)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GAIA_INC}
)
target_link_libraries(${PROJECT_NAME}_node gaia)

# Register the gaia_bot Composable Node as a ROS2 Component.
# You can either run the Node as an executable with `ros2 run` or put it into
# a Composable Node Container in your Python launch script.
# https://docs.ros.org/en/galactic/Tutorials/Composition.html
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "gaia_bot::GaiaBot"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_gaia_bot.cpp)
  set(TEST_GAIA_BOT_EXE test_gaia_bot)
  ament_add_gtest(${TEST_GAIA_BOT_EXE} ${TEST_SOURCES})

  target_add_gaia_generated_sources(${TEST_GAIA_BOT_EXE})
  target_include_directories(${TEST_GAIA_BOT_EXE}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      ${GAIA_INC}
  )
  target_link_libraries(${TEST_GAIA_BOT_EXE} ${PROJECT_NAME}_node gaia)
endif()

# Convenience macro for generating an ament package and installing launch files
# in the share directory.
ament_auto_package(INSTALL_TO_SHARE
  launch
)
