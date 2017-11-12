
# This version of the Makefile will work on any version of Ubuntu

CFLAGS=-g -std=gnu99 -fPIC -O2 `pkg-config opencv --cflags` `pkg-config python --cflags`
LDFLAGS=-g -fPIC `pkg-config opencv --libs` `pkg-config python --libs`

_raycaster.so: raycaster.o raycaster_wrap.o
	#sudo apt-get install libcv-dev libhighgui-dev libcvaux-dev
	$(CC) -shared -o _raycaster.so raycaster.o raycaster_wrap.o $(LDFLAGS)

raycaster.o: raycaster.c raycaster.h

raycaster_wrap.o: raycaster_wrap.c raycaster.h

raycaster.py raycaster_wrap.c: raycaster.i
	swig -python raycaster.i

clean:
	rm -f *.o raycaster.py raycaster_wrap.c _raycaster.so
