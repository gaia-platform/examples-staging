WFLAGS = -Wall -Werror
DEBUG_FLAG = -g
#OPTIMIZATION_FLAG = -O3
MISC_FLAGS = --std=c++17 
INC = -I/opt/gaia/include/ -I../gaia/ -I../gaia/slam/ -I../include/

CPPFLAGS = $(WFLAGS) $(DEBUG_FLAG) $(OPTIMIZATION_FLAG) $(MISC_FLAGS) $(INC)
