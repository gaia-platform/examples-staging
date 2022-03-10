include ../Makefile.inc

RULES_CPP = slam_rules.cpp
DB_NAME = slam

OBJS = slam_rules.o slam/gaia_slam.o

all: rules $(OBJS)

rules: 
	gaiac slam.ddl -g --db-name $(DB_NAME) -o $(DB_NAME)
	gaiat slam.ruleset -output $(RULES_CPP) -- -I/usr/lib/clang/10/include $(INC)
	

refresh: clean all

%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

clean:
	rm -f *.o $(RULES_CPP)
	rm -rf $(DB_NAME)

