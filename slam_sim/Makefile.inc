ROOT := $(dir $(lastword $(MAKEFILE_LIST)))

WFLAGS = -Wall -Werror
DEBUG_FLAG = -g

# Disable optimization for now -- using it generates a segfault at
#   gaia/slam/gaia_slam.cpp:734
#OPTIMIZATION_FLAG = -O1

MISC_FLAGS = --std=c++17 
INC = -I/opt/gaia/include/ -I$(ROOT)gaia/ -I$(ROOT)gaia/slam/ -I$(ROOT)include/

CPPFLAGS = $(WFLAGS) $(DEBUG_FLAG) $(MISC_FLAGS) $(INC)
