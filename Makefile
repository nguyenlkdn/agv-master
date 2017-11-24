IDIR =/usr/local/include/modbus/
CC=gcc
CFLAGS=-I$(IDIR)

ODIR=./
LDIR =/usr/local/lib/

LIBS=-lm -lpthread -lmodbus

_DEPS = config.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = modbus-rtu-server.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

server: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
