#define BENCHMARK "OSU MPIV Multi-threaded Latency Test"
/*
 * Copyright (C) 2002-2014 the Network-Based Computing Laboratory
 * (NBCL), The Ohio State University.
 *
 * Contact: Dr. D. K. Panda (panda@cse.ohio-state.edu)
 *
 * For detailed copyright and licensing information, please refer to the
 * copyright file COPYRIGHT in the top level OMB directory.
 */

#define USE_L1_MASK
#include "mpiv.h"
#include "helper.h"
#include <atomic>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <pthread.h>

#define MESSAGE_ALIGNMENT 64
#define MIN_MSG_SIZE 64
#define MAX_MSG_SIZE 64
#define MYBUFSIZE (MAX_MSG_SIZE + MESSAGE_ALIGNMENT)
#define SKIP_LARGE 100
#define LOOP_LARGE 1000
#define LARGE_MESSAGE_SIZE 8192

char* s_buf1;
char* r_buf1;
int skip = 1000;
int loop = 10000;

pthread_mutex_t finished_size_mutex;
pthread_cond_t finished_size_cond;

typedef struct thread_tag { int id; } thread_tag_t;

void send_thread(intptr_t arg);
void recv_thread(intptr_t arg);

#ifdef PACKAGE_VERSION
#define HEADER "# " BENCHMARK " v" PACKAGE_VERSION "\n"
#else
#define HEADER "# " BENCHMARK "\n"
#endif

#ifndef FIELD_WIDTH
#define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
#define FLOAT_PRECISION 2
#endif

int numprocs, provided, myid, err;
static int MAX_THREADS = 4096;
static int MIN_THREADS = 4096;
static int THREADS = 1;
static int WORKERS = 1;

int main(int argc, char* argv[]) {
  MPIV_Init(&argc, &argv);
  if (argc > 2) {
    MIN_THREADS = atoi(argv[1]);
    MAX_THREADS = atoi(argv[2]);
    WORKERS = atoi(argv[3]);
  }

  MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);

  if (numprocs != 2) {
    if (myid == 0) {
      fprintf(stderr, "This test requires exactly two processes\n");
    }

    MPIV_Finalize();
    return EXIT_FAILURE;
  }

  if (myid == 0)
    MPIV_Start_worker(1);
  else
    MPIV_Start_worker(WORKERS);

  MPIV_Finalize();
}

static int size = 0;

void main_task(intptr_t) {
  int i = 0;
  r_buf1 = (char*)MPIV_Alloc(MYBUFSIZE);
  s_buf1 = (char*)MPIV_Alloc(MYBUFSIZE);
  mv_thread* sr_threads = new mv_thread[MAX_THREADS];
  thread_tag_t* tags = new thread_tag_t[MAX_THREADS];

  if (myid == 0) {
    fprintf(stdout, HEADER);
    fprintf(stdout, "%-*s%*s\n", 10, "# Size", FIELD_WIDTH, "Latency (us)");
    fflush(stdout);
    for (THREADS = MIN_THREADS ; THREADS <= MAX_THREADS; THREADS <<= 1) {
        for (size = MIN_MSG_SIZE; size <= MAX_MSG_SIZE;
                size = (size ? size * 2 : 1)) {
            MPI_Barrier(MPI_COMM_WORLD);
            tags[i].id = 0;
            sr_threads[i] = MPIV_spawn(0, send_thread, (intptr_t)&tags[i]);
            MPIV_join(sr_threads[i]);
            MPI_Barrier(MPI_COMM_WORLD);
        }
    }
  } else {
      for (THREADS = MIN_THREADS ; THREADS <= MAX_THREADS; THREADS <<= 1) {
          for (size = MIN_MSG_SIZE; size <= MAX_MSG_SIZE;
                  size = (size ? size * 2 : 1)) {
              MPI_Barrier(MPI_COMM_WORLD);
              for (i = 0; i < THREADS; i++) {
                  sr_threads[i] =
                      MPIV_spawn(i % WORKERS, recv_thread, (intptr_t) i);
              }
              for (i = 0; i < THREADS; i++) {
                  MPIV_join(sr_threads[i]);
              }
              MPI_Barrier(MPI_COMM_WORLD);
          }
      }
  }
  MPIV_Free(r_buf1);
  MPIV_Free(s_buf1);
}

void recv_thread(intptr_t arg) {
  int i, val, align_size;
  char *s_buf, *r_buf;
  val = (int) (arg); 

  align_size = MESSAGE_ALIGNMENT;

  s_buf = (char*)(((unsigned long)s_buf1 + (align_size - 1)) / align_size *
                  align_size);
  r_buf = (char*)(((unsigned long)r_buf1 + (align_size - 1)) / align_size *
                  align_size);

  if (size > LARGE_MESSAGE_SIZE) {
    loop = LOOP_LARGE;
    skip = SKIP_LARGE;
  }

  loop = std::max(THREADS, loop);

  /* touch the data */
  for (i = 0; i < size; i++) {
    s_buf[i] = 'a';
    r_buf[i] = 'b';
  }

  for (i = val; i < (loop + skip); i += THREADS) {
    MPIV_Recv(r_buf, size, MPI_CHAR, 0, i, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    MPIV_Send(s_buf, size, MPI_CHAR, 0, i, MPI_COMM_WORLD);
    fflush(stdout);
  }
  // sleep(1);
}

void send_thread(intptr_t) {
  int i, align_size;
  char *s_buf, *r_buf;
  double t_start = 0, t_end = 0, t = 0, latency;
  align_size = MESSAGE_ALIGNMENT;

  s_buf = (char*)(((unsigned long)s_buf1 + (align_size - 1)) / align_size *
                  align_size);
  r_buf = (char*)(((unsigned long)r_buf1 + (align_size - 1)) / align_size *
                  align_size);

  if (size > LARGE_MESSAGE_SIZE) {
    loop = LOOP_LARGE;
    skip = SKIP_LARGE;
  }
  loop = std::max(THREADS, loop);

  /* touch the data */
  for (i = 0; i < size; i++) {
    s_buf[i] = 'a';
    r_buf[i] = 'b';
  }

#if 1
  for (i = 0; i < loop + skip; i++) {
    if (i == skip) {
      t_start = MPI_Wtime();
    }

    MPIV_Send(s_buf, size, MPI_CHAR, 1, i, MPI_COMM_WORLD);
    MPIV_Recv(r_buf, size, MPI_CHAR, 1, i, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
  }
#endif

  t_end = MPI_Wtime();
  t = t_end - t_start;

  latency = (t)*1.0e6 / (2.0 * loop);
  std::cout << THREADS << "\t" << size << "\t" << latency << std::endl;
}

/* vi: set sw=4 sts=4 tw=80: */
