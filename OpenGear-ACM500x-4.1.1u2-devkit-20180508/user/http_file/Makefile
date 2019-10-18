
BIN  = http_file
OBJS = http_file.o

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS$(LDLIBS_$@))

romfs:
	$(ROMFSINST) /bin/$(BIN)

clean:
	-rm -f $(BIN) *.elf *.gdb *.o

