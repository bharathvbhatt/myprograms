
BIN  = run
OBJS = run.o

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS$(LDLIBS_$@))

romfs:
	$(ROMFSINST) /bin/$(BIN)

clean:
	-rm -f $(BIN) *.elf *.gdb *.o

