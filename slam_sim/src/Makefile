include ../Makefile.inc

# Make sure that we're not using g++. If so then environment variables
#   aren't correctly set.
GXX_NAME = g++
ifeq ($(CXX), $(GXX_NAME))
    $(error Using g++. Environment variables are not correctly configured)
endif


TARGET = slam_sim

OBJS = main.o analyze.o occupancy.o rule_api.o  path_map.o support.o
EXT_OBJS = ../gaia/slam_rules.o ../gaia/gaia_slam.o
LIBS = $(LIB_SLAM_UTILS) /usr/local/lib/libgaia.so -pthread -lcurl

all: $(OBJS) 
	cd utils && make
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

run:
	./$(TARGET) -m ../data/map.json -x -3.0 -y -3.0

drun:
	gdb --args ./slam_sim -m ../data/map.json -x -3.0 -y -3.0


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	cd utils && make clean
	rm -f *.o $(TARGET)
	rm -rf logs


