include ../Makefile.inc

RULES_CPP = slam_rules.cpp
DB_NAME = slam

OBJS = slam_rules.o slam/gaia_slam.o

all: rules all_procedure_subshell

gaiac:
	gaiac slam.ddl -g --db-name $(DB_NAME) -o $(DB_NAME)

rules: gaiac
	gaiat slam.ruleset -output $(RULES_CPP) -- -I/usr/lib/clang/10/include $(INC)

# when freshly making, rules.cpp may not exist (e.g., if 'make clean'
# 	was just run). drop into subshell for making local procedural files
# 	as that will induce a scan of local files
all_procedure_subshell:
	make all_procedure

all_procedure: $(OBJS)

refresh: clean all

%.o: %.cpp 
	$(CXX) $(INC) -c $(CPPFLAGS) $< 

clean:
	rm -f *.o $(RULES_CPP)
	rm -rf $(DB_NAME)

