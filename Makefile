.PHONY: all clean install tests

include config.mk

CC = gcc
CFLAGS ?= -g3 -ggdb -O3 -Wall -Wextra # -qopt-report=4 -qopt-report-phase ipo
AR ?= ar
RANLIB ?= ranlib
PREFIX ?= /usr

SRCDIR = ./src
OBJDIR ?= ./obj

EXTRA = -DAFF_DEBUG -DUSE_AFFI #-DUSE_MINI_PSM2# -DUSE_ABT

ifeq (,$(wildcard $(OBJDIR)))
$(shell mkdir -p $(OBJDIR))
endif

ifeq (,$(wildcard $(OBJDIR)/pmi))
$(shell mkdir -p $(OBJDIR)/pmi)
endif

ifeq (,$(wildcard $(OBJDIR)/dreg))
$(shell mkdir -p $(OBJDIR)/dreg)
endif

INCLUDE = -I./src/include -I./include
CFLAGS += -fPIC -fvisibility=hidden -std=gnu99 $(INCLUDE) $(EXTRA) -D_GNU_SOURCE -pthread

IBV_DIR = /opt/ofed/
PSM_DIR = $(HOME)/libpsm2/usr/

CFLAGS += $(IBV_INC)
LDFLAGS += -shared -Lstatic

### FOR USING PAPI ##
# PAPI_LIB = $(TACC_PAPI_LIB) -lpapi
# PAPI_INC = $(TACC_PAPI_INC)
# CFLAGS += -I$(PAPI_INC)
# LDFLAGS += -L$(PAPI_LIB)

### BEGIN -- ENDPOINT OPTIONS SELECTIONS
## Addressing mode.
ifneq (,$(findstring explicit,$(LCI_EP_AR)))
CFLAGS += -DLCI_SERVER_HAS_EXP
endif

ifneq (,$(findstring dynamic,$(LCI_EP_AR)))
CFLAGS += -DLCI_SERVER_HAS_DYN 
endif

ifneq (,$(findstring immediate,$(LCI_EP_AR)))
CFLAGS += -DLCI_SERVER_HAS_IMM
endif

CFLAGS += -DLCI_MAX_EP=$(LCI_MAX_EP)
CFLAGS += -DLCI_MAX_DEV=$(LCI_MAX_DEV)

## Completion events.
ifneq (,$(findstring am,$(LCI_EP_CE)))
CFLAGS += -DLCI_SERVER_HAS_AM
endif

ifneq (,$(findstring sync,$(LCI_EP_CE)))
CFLAGS += -DLCI_SERVER_HAS_SYNC
endif

ifneq (,$(findstring cq,$(LCI_EP_CE)))
CFLAGS += -DLCI_SERVER_HAS_CQ
endif

ifneq (,$(findstring glob,$(LCI_EP_CE)))
CFLAGS += -DLCI_SERVER_HAS_GLOB
endif

### END -- ENDPOINT OPTIONS SELECTIONS

ifeq ($(LCI_SERVER_DEBUG), yes)
CFLAGS += -DLCI_SERVER_DEBUG
endif

ifeq ($(LCI_SERVER_INLINE), yes)
CFLAGS += -DLCI_SERVER_INLINE
endif

ifeq ($(LCI_SERVER), ofi)
	CFLAGS += -DLCI_USE_SERVER_OFI -DAFF_DEBUG
	LDFLAGS += -lfabric
endif

ifeq ($(LCI_SERVER), ibv)
	CFLAGS += -DLCI_USE_SERVER_IBV -DAFF_DEBUG -I$(IBV_DIR)/include
	LDFLAGS += -L$(IBV_DIR)/lib -libverbs
endif

ifeq ($(LCI_SERVER), psm)
	CFLAGS += -DLCI_USE_SERVER_PSM -DAFF_DEBUG -I$(PSM_DIR)/include
	LDFLAGS += -L$(PSM_DIR)/lib -lpsm2
endif

SYNC_FLAG = sync_flag.o
# SYNC_ABT = sync_abt.o
# CFLAGS += -labt -L$(HOME)/abt/lib -I$(HOME)/abt/include

SYNC = $(SYNC_FLAG)

#COMM = lc.o medium.o short.o long.o tag.o cq.o misc.o ep.o lcrq.o pool.o hashtable.o coll.o glob.o
DREG = dreg/dreg.o dreg/avl.o
PMI = pm.o pmi/simple_pmi.o pmi/simple_pmiutil.o
COMM = lci.o plist.o config.o pool.o misc.o lcrq.o pt2pt.o 1sided.o cq.o mt.o $(SYNC)

ifeq ($(LCI_USE_DREG), yes)
LIBOBJ += $(addprefix $(OBJDIR)/, $(DREG))
CFLAGS += -DUSE_DREG
endif

OBJECTS = $(addprefix $(OBJDIR)/, $(COMM))

LIBOBJ += $(OBJECTS) $(addprefix $(OBJDIR)/, $(FCONTEXT)) $(addprefix $(OBJDIR)/, $(PMI))

LIBRARY = liblci.so
ARCHIVE = liblci.a
PKGCONFIG = liblci.pc

all: $(LIBRARY) $(ARCHIVE)

install: all
	mkdir -p $(PREFIX)/bin
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/lib/pkgconfig
	mkdir -p $(PREFIX)/include
	cp lcrun $(PREFIX)/bin
	cp -R include/* $(PREFIX)/include
	cp $(ARCHIVE) $(PREFIX)/lib
	cp $(LIBRARY) $(PREFIX)/lib
	cp $(PKGCONFIG) $(PREFIX)/lib/pkgconfig

$(OBJDIR)/%.o: $(SRCDIR)/$(notdir %.c) $(SRCDIR)/include/config.h
	$(CC) $(CFLAGS) -c $< -o $@

$(LIBRARY): $(LIBOBJ)
	$(CC) $(LDFLAGS) $(LIBOBJ) $(MALLOC) -o $(LIBRARY)

$(ARCHIVE): $(LIBOBJ)
	@rm -f $(ARCHIVE)
	$(AR) q $(ARCHIVE) $(LIBOBJ) $(MALLOC)
	$(RANLIB) $(ARCHIVE)

$(SRCDIR)/include/config.h:: $(SRCDIR)/include/config.h.mk
	cp $< $@

mpiv.a: obj/mpiv.o
	$(AR) q mpiv.a obj/mpiv.o $(LIBOBJ)
	$(RANLIB) mpiv.a

clean:
	rm -rf $(LIBOBJ) $(OBJDIR)/* $(SRCDIR)/include/config.h $(ARCHIVE) $(LIBRARY) mpiv.a
	$(MAKE) clean -C examples
	$(MAKE) clean -C benchmarks

tests:
	$(MAKE) -C tests && ./tests/all_test
