include ../../Makefile.inc

TEST_TARGETS = test_line_segment 

OBJS = line_segment.o blob_cache.o


all: $(OBJS) 
	ar rcs $(LIB_SLAM_UTILS) *.o

test: $(TEST_TARGETS)

test_line_segment: line_segment.cpp 
	$(CXX) $(CPPFLAGS) line_segment.cpp -o test_line_segment -DTEST_LINE_SEGMENT


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	rm -f *.o *.a $(TEST_TARGETS)


