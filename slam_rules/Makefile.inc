ROOT := $(dir $(lastword $(MAKEFILE_LIST)))

WFLAGS = -Wall -Werror
DEBUG_FLAG = -g
#OPTIMIZATION_FLAG = -O3
MISC_FLAGS = --std=c++17 
INC = -I/opt/gaia/include/ -I$(ROOT)gaia/ -I$(ROOT)gaia/slam/ -I$(ROOT)include/

CPPFLAGS = $(WFLAGS) $(DEBUG_FLAG) $(OPTIMIZATION_FLAG) $(MISC_FLAGS) $(INC)
