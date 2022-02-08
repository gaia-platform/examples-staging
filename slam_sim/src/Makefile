include ../Makefile.inc

# Make sure that we're not using g++. If so then environment variables
#   aren't correctly set.
GXX_NAME = g++
ifeq ($(CXX), $(GXX_NAME))
    $(error Using g++. Environment variables are not correctly configured)
endif

TARGET = slam_sim

TEST_TARGETS = test_line_segment test_analyze

OBJS = line_segment.o analyze.o slam_sim.o comm_thread.o
EXT_OBJS = 
LIBS = /usr/local/lib/libgaia.so -lcurl -pthread

all: $(OBJS) 
	$(CXX) $(OBJS) -o $(TARGET) $(LIBS)

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


