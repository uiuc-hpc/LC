TARGET = test

all: $(TARGET)

test: pmi.a test.c
	$(CC) -Wall -g3 -O3 -std=gnu99 test.c pmi.a -o test

pmi.a: simple_pmi.o simple_pmiutil.o
	ar rcs $@ $^

%.o: %.c
	$(CC) -std=gnu99 -c $^ -g3 -O3 -Wall -D_GNU_SOURCE

clean:
	rm -rf *.a *.o $(TARGET)
