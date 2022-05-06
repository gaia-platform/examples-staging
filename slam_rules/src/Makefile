include ../Makefile.inc

# Make sure that we're not using g++. If so then environment variables
#   aren't correctly set.
GXX_NAME = g++
ifeq ($(CXX), $(GXX_NAME))
    $(error Using g++. Environment variables are not correctly configured)
endif


TARGET = slam_sim
TEST_TARGETS = test_analyze test_line_segment test_blob_cache

CORE_OBJS = analyze.o blob_cache.o line_segment.o occupancy.o \
    rule_api.o  path_map.o support.o globals.o txn.o
OBJS = main.o $(CORE_OBJS)
EXT_OBJS = ../gaia/slam_rules.o ../gaia/gaia_slam.o
LIBS = /usr/local/lib/libgaia.so -pthread -lcurl

all: $(OBJS) 
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

test: $(TEST_TARGETS)

test_analyze: test_analyze.cpp analyze.cpp $(CORE_OBJS)
	$(CXX) $(CPPFLAGS) test_analyze.cpp -o test_analyze \
        $(CORE_OBJS) $(EXT_OBJS) $(LIBS)

test_line_segment: line_segment.cpp 
	$(CXX) $(CPPFLAGS) test_line_segment.cpp line_segment.cpp -o test_line_segment

test_blob_cache: blob_cache.cpp 
	$(CXX) $(CPPFLAGS) test_blob_cache.cpp blob_cache.cpp -o test_blob_cache

run:
	./$(TARGET) -m ../data/maze3.json -x 4.5 -y 6.0
	#./$(TARGET) -m ../data/maze2.json -x -3.5 -y 1.5
	#./$(TARGET) -m ../data/maze.json -x -4.5 -y 1.25

drun:
	gdb --args ./slam_sim -m ../data/maze3.json -x 4.5 -y 6.0
	#gdb --args ./slam_sim -m ../data/maze.json -x -4.5 -y 1.25


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	rm -f *.o $(TARGET) *.pnm
	rm -rf logs

clean_images:
	rm -f *.pnm
