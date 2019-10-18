
BIN  = AIS
OBJS = main.o datastruct.o file_handling.o tcp_driver.o tcp_main.o usb_init.o usbserial_main.o
        
GIT_VERSION := $(shell git --no-pager describe --tags --always)
BUILD_DATE  := $(shell date --iso=seconds)

CFLAGS := -Wall -Wextra -pedantic -O\
	-DGIT_VERSION=\"$(GIT_VERSION)\"\
	-DBUILD_DATE=\"$(BUILD_DATE)\"

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS$(LDLIBS_$@)) -lpthread

romfs:
	$(ROMFSINST) /bin/$(BIN)

clean:
	-rm -f $(BIN) *.elf *.gdb *.o

