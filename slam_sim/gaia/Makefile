# Copyright (c) Gaia Platform LLC
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.

# The purpose of this makefile is to simply execute gaiac and gaiat
#   to make sure there are not errors (without executing higher level
#   make). The single target then cleans up after itself.

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

