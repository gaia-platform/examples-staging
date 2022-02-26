cmake_minimum_required(VERSION 3.17)

project(access_control_app VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(/opt/gaia/cmake/gaia.cmake)

find_package(Threads REQUIRED)
find_package(aws-crt-cpp REQUIRED)

add_compile_options(-g)

# Generate C++ headers from the .ddl schema file.
process_schema(
  DDL_FILE ${PROJECT_SOURCE_DIR}/src/access_control.ddl
  DATABASE_NAME "access_control"
)

# Translate the declarative .ruleset file into a C++ source file.
translate_ruleset(
  RULESET_FILE ${PROJECT_SOURCE_DIR}/src/access_control.ruleset
  CLANG_PARAMS -I ${PROJECT_SOURCE_DIR}/include
)

add_executable(access_control
  src/main.cpp
  src/helpers.cpp
  src/actions.cpp
  src/communication.cpp
)

target_add_gaia_generated_sources(access_control)
add_dependencies(access_control translate_access_control_ruleset)

target_include_directories(access_control
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${GAIA_INC}
)

target_link_libraries(access_control
  PRIVATE
    ${GAIA_LIB}
    Threads::Threads
    AWS::aws-crt-cpp
)
