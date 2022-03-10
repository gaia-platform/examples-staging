#############################################
# Copyright (c) 2022 Gaia Platform LLC
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.
#############################################

cmake_minimum_required(VERSION 3.16)

# Set the project name.
project(frequent_flyer)

set(CMAKE_CXX_STANDARD 17)

# We need pthreads support.
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include("/opt/gaia/cmake/gaia.cmake")

# Default compiler/linker flags.
add_compile_options(-c -Wall -Wextra)

set(FREQUENT_FLYER_DDL ${PROJECT_SOURCE_DIR}/src/frequent_flyer.ddl)
set(FREQUENT_FLYER_RULESET ${PROJECT_SOURCE_DIR}/src/frequent_flyer.ruleset)

# --- Generate Direct Access classes from DDL---
process_schema(
  DDL_FILE ${FREQUENT_FLYER_DDL}
  DATABASE_NAME frequent_flyer
)

# -- Translate ruleset into CPP --
translate_ruleset(RULESET_FILE ${FREQUENT_FLYER_RULESET})

add_executable(frequent_flyer
  src/frequent_flyer.cpp
)

target_add_gaia_generated_sources(frequent_flyer)
target_include_directories(frequent_flyer PRIVATE ${GAIA_INC})
target_link_directories(frequent_flyer PRIVATE ${GAIA_LIB_DIR})
target_link_libraries(frequent_flyer PRIVATE gaia rt Threads::Threads)
