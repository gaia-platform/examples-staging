
all:
	#mkdir -p build && cd build && cmake .. && make -j 4
	cd src && make

run: 
	cd src && make run

# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
refresh_db:
	gaiac gaia/slam.ddl --db-name slam -g -o /tmp/slam

refresh: clean all

clean:
	rm -Rf build

