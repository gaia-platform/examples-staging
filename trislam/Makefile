# Copyright (c) Gaia Platform LLC
#
# Use of this source code is governed by the MIT
# license that can be found in the LICENSE.txt file
# or at https://opensource.org/licenses/MIT.

# 'make all' executes cmake in 'build' directory. 
# 'make clean' deletes the cmake build/ folder.
# 'make refresh_db' runs gaiac to clean out old content.
# 'make refresh' runs 'make clann all'

all:
	#mkdir -p build && cd build && cmake .. && make
	mkdir -p build && cd build && cmake .. && make -j 4

run: 
	cd src && make run

# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
refresh_db:
	gaiac gaia/slam.ddl --db-name slam -g -o /tmp/slam

refresh: clean all

clean:
	rm -Rf build

