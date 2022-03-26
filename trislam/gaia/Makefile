
RULES_CPP = slam_rules.cpp
DB_NAME = slam
INC = -I/opt/gaia/include/ -Islam/ -I../include/

OBJS = slam_rules.o slam/gaia_slam.o

all:
	gaiac slam.ddl -g --db-name $(DB_NAME) -o $(DB_NAME)
	gaiat slam.ruleset -output $(RULES_CPP) -- -I/usr/lib/clang/10/include $(INC)

refresh: clean all

clean:
	rm -f *.o $(RULES_CPP)
	rm -rf $(DB_NAME)

