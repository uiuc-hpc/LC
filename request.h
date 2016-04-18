#ifndef REQUEST_H_
#define REQUEST_H_

#include "mpiv.h"

struct mpiv_packet;

extern __thread fult* __fulting;

struct MPIV_Request {
  inline MPIV_Request(int rank_, int tag_)
      : rank(rank_), tag(tag_), sync(__fulting), done_(false) {}
  inline MPIV_Request(void* buffer_, int size_, int rank_, int tag_)
      : buffer(buffer_),
        size(size_),
        rank(rank_),
        tag(tag_),
        sync(__fulting),
        done_(false) {};
  void* buffer;
  int size;
  int rank;
  int tag;
  fult* sync;
  bool done_;
} __attribute__((aligned(64)));

inline void MPIV_Wait(MPIV_Request* req) {
  if (xunlikely(req->done_)) return;
#ifdef USE_WORKER_WAIT
  req->sync->origin()->wait(req->done_);
#else
  req->sync->wait();
#endif
}

inline void MPIV_Signal(MPIV_Request* req) {
#ifdef USE_WORKER_WAIT
  req->sync->origin()->resume(req->done_);
#else
  req->sync->resume();
#endif
}

// FIXME(danghvu): DO NOT USE THIS
void MPIV_Waitall(int count, MPIV_Request request[], MPI_Status*) {
  for (int i=0; i<count; i++) {
    MPIV_Wait(&request[i]);
  }
}


#endif
