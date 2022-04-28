include ../../Makefile.inc

TEST_TARGETS = test_line_segment test_blob_cache

OBJS = line_segment.o blob_cache.o


all: $(OBJS) 
	ar rcs $(LIB_SLAM_UTILS) *.o

test: $(TEST_TARGETS)

test_line_segment: line_segment.cpp 
	$(CXX) $(CPPFLAGS) test_line_segment.cpp line_segment.cpp -o zline_segment

test_blob_cache: blob_cache.cpp 
	$(CXX) $(CPPFLAGS) test_blob_cache.cpp blob_cache.cpp -o zblob_cache


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	rm -f *.o *.a $(TEST_TARGETS) z*


