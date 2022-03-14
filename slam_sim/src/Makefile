include ../Makefile.inc

TARGET = slam_sim

TEST_TARGETS = test_analyze 

OBJS = main.o paths.o navigation.o analyze.o
EXT_OBJS = $(ROOT)/gaia/slam_rules.o $(ROOT)/gaia/gaia_slam.o

LIBS = $(LIB_SLAM_UTILS) /usr/local/lib/libgaia.so -lcurl -pthread


all: $(OBJS) 
	echo $(ROOT)
	cd utils && make
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

run: refresh_db
	./slam_sim -m ../data/map.json

drun: refresh_db
	gdb --args ./slam_sim -m ../data/map.json

# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
refresh_db:
	cd ../gaia/ && make gaiac


test: $(TEST_TARGETS)


test_analyze: analyze.cpp 
	$(CXX) $(CPPFLAGS) analyze.cpp -o test_analyze -DTEST_ANALYZE $(LIBS)


%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	cd utils && make clean
	rm -f *.o $(TARGET) $(TEST_TARGETS)
	rm -rf logs


