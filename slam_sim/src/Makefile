
all:
	mkdir -p ../build && cd ../build/ && cmake .. && make

run: refresh_db
	../build/slam_sim -m ../data/map.json -x 4.0 -y 0.6

drun: refresh_db
	gdb --args ../build/slam_sim -m ../data/map.json -x 4.0 -y 0.6


# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
# NOTE: this may be implemented, but there may be a problem w/ that
#   in regards to connected records
refresh_db:
	gaiac ../gaia/slam.ddl --db-name slam -g -o /tmp/slam


########################################################################
# From original content. Keep this around until building the test
#   code is migrated to cmake.

include ../Makefile.inc


TEST_TARGETS = test_analyze 


LIBS = /usr/local/lib/libgaia.so -lcurl -pthread


test: $(TEST_TARGETS)


test_analyze: analyze.cpp 
	$(CXX) $(CPPFLAGS) utils/*.cpp analyze.cpp -o test_analyze -DTEST_ANALYZE $(LIBS)


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean test

clean:
	rm -f *.o $(TEST_TARGETS)

