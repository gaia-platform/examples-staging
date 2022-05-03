include ../Makefile.inc

# Make sure that we're not using g++. Use that as a proxy to determine
#   if environment variables are set, as we should be using clang if so.
GXX_NAME = g++
ifeq ($(CXX), $(GXX_NAME))
    $(error Using g++. Environment variables are not correctly configured)
endif


TARGET = slam_sim
TEST_TARGETS = test_analyze

CORE_OBJS = analyze.o occupancy.o navigation.o path_map.o support.o \
    globals.o txn.o line_segment.o
OBJS = main.o $(CORE_OBJS)
EXT_OBJS = ../gaia/gaia_slam.o
LIBS = /usr/local/lib/libgaia.so -pthread -lcurl

all: $(OBJS)
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

test: $(TEST_TARGETS)

test_analyze: test_analyze.cpp analyze.cpp $(CORE_OBJS)
	$(CXX) $(CPPFLAGS) test_analyze.cpp -o test_analyze \
        $(CORE_OBJS) $(EXT_OBJS) $(LIBS)


run:
	./$(TARGET) -m ../data/maze3.json -x 4.5 -y 6.0
	#./$(TARGET) -m ../data/maze2.json -x -3.5 -y 1.5
	#./$(TARGET) -m ../data/maze.json -x -4.5 -y 1.25

drun:
	gdb --args ./slam_sim -m ../data/maze3.json -x 4.5 -y 6.0
	#gdb --args ./slam_sim -m ../data/maze.json -x -4.5 -y 1.25


%.o: %.cpp
	$(CXX) $(INC) -c $(CPPFLAGS) $(OPTIMIZATION_FLAG) $<

refresh: clean all

clean:
	rm -f *.o $(TARGET) *.pnm
	rm -rf logs

clean_images:
	rm -f *.pnm
