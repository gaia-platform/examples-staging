
all:
	#mkdir -p ../build && cd ../build/ && cmake .. && make 
	mkdir -p ../build && cd ../build/ && cmake .. && make -j 4

run: refresh_db
	../build/slam_demo -m ../data/map.json -p ../data/path.txt

drun: refresh_db
	gdb --args ../build/slam_demo -m ../data/map.json -p ../data/path.txt


# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
# NOTE: this may be implemented, but there may be a problem w/ that
#   in regards to connected records
refresh_db:
	gaiac ../gaia/slam.ddl --db-name slam -g -o /tmp/slam

clean:
	rm -f *.pgm *.o
