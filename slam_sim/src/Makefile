include ../Makefile.inc

# Make sure that we're not using g++. If so then environment variables
#   aren't correctly set.
GXX_NAME = g++
ifeq ($(CXX), $(GXX_NAME))
    $(error Using g++. Environment variables are not correctly configured)
endif


TARGET = slam_sim

OBJS = main.o analyze.o navigation.o occupancy.o rule_api.o 
EXT_OBJS = ../gaia/slam_rules.o ../gaia/gaia_slam.o
LIBS = /usr/local/lib/libgaia.so -pthread -lcurl

all: $(OBJS) 
	$(CXX) $(OBJS) $(EXT_OBJS) -o $(TARGET) $(LIBS)
	#cp $(TARGET) ../runtime/sim_shell

%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

refresh: clean all

clean:
	rm -f *.o $(TARGET)
	rm -rf logs


