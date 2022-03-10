include ../Makefile.inc

TARGET = slam_sim

TEST_TARGETS = test_line_segment test_analyze

OBJS = line_segment.o analyze.o slam_sim.o comm_thread.o anchor_description.o
EXT_OBJS = ../gaia/slam_rules.o ../gaia/gaia_slam.o

LIBS = /usr/local/lib/libgaia.so -lcurl -pthread

all: $(OBJS) 
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

test: $(TEST_TARGETS)

test_analyze: analyze.cpp line_segment.o
	$(CXX) $(CPPFLAGS) analyze.cpp line_segment.o -o test_analyze -DTEST_ANALYZE

test_line_segment: line_segment.cpp 
	$(CXX) $(CPPFLAGS) line_segment.cpp -o test_line_segment -DTEST_LINE_SEGMENT


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	rm -f *.o $(TARGET) $(TEST_TARGETS)
	rm -rf logs


