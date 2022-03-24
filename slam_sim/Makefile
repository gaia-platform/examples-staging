
all:
	mkdir -p build && cd build && cmake .. && make -j 4

# At present there's no method to delete the contents of the database
#   from w/in the program, so add a way to do so outside.
refresh_db:
	cd gaia/ && make gaiac

refresh: clean2 all2

clean:
	rm -Rf build


all_old:
	cd gaia; make all
	cd src; make all

refresh_old: clean all

clean_old:
	cd gaia; make clean
	cd src; make clean

