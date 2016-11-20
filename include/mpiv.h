#ifndef MPIV_MPIV_H_
#define MPIV_MPIV_H_

#include <mpi.h>
#include "mv.h"
#include "request.h"
#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>

typedef boost::interprocess::basic_managed_external_buffer<
    char, boost::interprocess::rbtree_best_fit<
              boost::interprocess::mutex_family, void*, 64>,
    boost::interprocess::iset_index>
    mbuffer;

extern mv_engine* mv_hdl;
extern mbuffer heap_segment;

MV_INLINE void MPIV_Recv(void* buffer, int count, MPI_Datatype datatype, int rank,
               int tag, MPI_Comm, MPI_Status*) {
  int size;
  MPI_Type_size(datatype, &size);
  mv_recv(mv_hdl, buffer, size * count, rank, tag);
}

MV_INLINE void MPIV_Send(void* buffer, int count, MPI_Datatype datatype, int rank,
               int tag, MPI_Comm) {
  int size;
  MPI_Type_size(datatype, &size);
  mv_send(mv_hdl, buffer, size * count, rank, tag);
}

MV_INLINE void MPIV_Irecv(void* buffer, int count, MPI_Datatype datatype, int rank,
                int tag, MPI_Comm, MPIV_Request* s) {
  int size;
  MPI_Type_size(datatype, &size);
  mv_irecv(mv_hdl, buffer, size * count, rank, tag, s);
}

MV_INLINE void MPIV_Isend(const void* buf, int count, MPI_Datatype datatype, int rank,
                int tag, MPI_Comm, MPIV_Request* req) {
  int size;
  MPI_Type_size(datatype, &size);
  mv_isend(mv_hdl, buf, size * count, rank, tag, req);
}

MV_INLINE void MPIV_Waitall(int count, MPIV_Request* req, MPI_Status*) {
  mv_waitall(mv_hdl, count, req);
}

MV_INLINE void MPIV_Init(int* argc, char*** args) {
  size_t heap_size = 1024 * 1024 * 1024;
  mv_open(argc, args, heap_size, &mv_hdl);
  heap_segment = std::move(mbuffer(boost::interprocess::create_only,
        mv_heap_ptr(mv_hdl), heap_size));
}

MV_INLINE void MPIV_Finalize() {
  mv_close(mv_hdl);
}

MV_INLINE void* MPIV_Alloc(int size) {
  return heap_segment.allocate(size);
}

MV_INLINE void MPIV_Free(void* ptr) {
  return heap_segment.deallocate(ptr);
}

#endif
