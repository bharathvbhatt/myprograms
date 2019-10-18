
BIN  = timer
OBJS = timer.o

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS$(LDLIBS_$@)) -lpthread

romfs:
	$(ROMFSINST) /bin/$(BIN)

clean:
	-rm -f $(BIN) *.elf *.gdb *.o

