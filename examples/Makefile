include ../config.mk
FULT=$(WORK)/fult

CC = gcc
LD = $(CC)
TARGET = tag queue

EXTRA = -DUSE_AFFI

CFLAGS += -g3 -ggdb -O3 -Wall -Wextra -std=gnu99 -pthread -I../include -I../ $(EXTRA) -D_GNU_SOURCE
LDFLAGS += -O3 -Wl,-rpath=$(abspath ../) -Wl,-z,now -Wl,-z,relro -pthread

LIB = -L.. -llwci
LIB_MPIV = ../mpiv.a

CXXFLAGS = -g3 -O3 -std=c++11

all: $(TARGET)

%: %.c
	$(CC) $(CFLAGS) -c $^
	$(LD) $(LDFLAGS) $@.o $(LIB) -o $@

pingpong: pingpong.c
	$(CC) $(CFLAGS) -c $^ -I$(FULT)/include
	$(LD) $(LDFLAGS) $@.o $(LIB_MPIV) -o $@ $(FULT)/libfult.a

pingpong_waitall: pingpong_waitall.c
	$(CC) $(CFLAGS) -c $^
	$(LD) $(LDFLAGS) $@.o $(LIB_MPIV) -o $@

%_abt: %.c
	$(CC) $(CFLAGS) -c $^ -DUSE_ABT
	$(LD) $(LDFLAGS) $*.o $(LIB_MPIV) -o $@

%_pth: %.c
	$(CC) $(CFLAGS) -c $^ -DUSE_PTH
	$(LD) $(LDFLAGS) $*.o $(LIB_MPIV) -o $@

clean:
	rm -rf $(TARGET) *.o
