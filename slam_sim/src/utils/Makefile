include ../../Makefile.inc

TEST_TARGETS = test_line_segment 


test: $(TEST_TARGETS)

test_line_segment: line_segment.cpp 
	$(CXX) $(CPPFLAGS) line_segment.cpp -o test_line_segment -DTEST_LINE_SEGMENT


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean test

clean:
	rm -f *.o *.a $(TEST_TARGETS)


