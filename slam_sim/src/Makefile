include ../Makefile.inc

TARGET = slam_sim

TEST_TARGETS = test_analyze 

OBJS = paths.o navigation.o analyze.o main.o
EXT_OBJS = $(ROOT)/gaia/slam_rules.o $(ROOT)/gaia/gaia_slam.o

LIBS = $(LIB_SLAM_UTILS) /usr/local/lib/libgaia.so -lcurl -pthread


all: $(OBJS) 
	echo $(ROOT)
	cd utils && make
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)

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


