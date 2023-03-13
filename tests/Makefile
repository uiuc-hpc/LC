GTEST_DIR = ./googletest/googletest/
GTEST_FLAGS = -std=c++11 -isystem ${GTEST_DIR}/include -I${GTEST_DIR} -pthread -O3

LC_FLAGS = -I../include -I../src/include
LC_LINK = ../liblci.a -libverbs

GXX = g++

TARGET = all_test
DEPS = $(shell find . -name '*.h')

all: $(TARGET)

clean:
	rm -rf $(TARGET)

libgtest.a:
	$(GXX) $(GTEST_FLAGS) -c ${GTEST_DIR}/src/gtest-all.cc
	ar -rv libgtest.a gtest-all.o

%_test: %.cxx libgtest.a $(DEPS)
	$(GXX) $(LC_FLAGS) $(GTEST_FLAGS) $< libgtest.a $(LC_LINK) -o $@
