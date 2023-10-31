#ifndef SERVER_OFI_H_
#define SERVER_OFI_H_

#include "config.h"

#include <mpi.h>
#include <rdma/fabric.h>
#include <rdma/fi_domain.h>
#include <rdma/fi_endpoint.h>
#include <stdlib.h>
#include <string.h>

#include <rdma/fi_cm.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_rma.h>

#include "mv/macro.h"
#include "dreg.h"

// #define SERVER_FI_DEBUG

#ifdef SERVER_FI_DEBUG
#define FI_SAFECALL(x)                                                    \
  {                                                                       \
    int err = (x);                                                        \
    if (err) {                                                            \
      printf("err : %s (%s:%d)\n", fi_strerror(err), __FILE__, __LINE__); \
      MPI_Abort(MPI_COMM_WORLD, err);                                     \
    }                                                                     \
  }                                                                       \
  while (0)                                                               \
    ;

#else
#define FI_SAFECALL(x) \
  {                    \
    (x);               \
  }
#endif

#define ALIGNMENT (lcg_page_size)
#define ALIGNEDX(x) \
  (void*)((((uintptr_t)x + ALIGNMENT - 1) / ALIGNMENT * ALIGNMENT))
#define MAX_CQ_SIZE (16 * 1024)
#define MAX_POLL 8

typedef struct ofi_server {
  struct fi_info* fi;
  struct fid_fabric* fabric;
  struct fid_domain* domain;
  struct fid_ep* ep;
  struct fid_cq* scq;
  struct fid_cq* rcq;
  struct fid_mr* mr_heap;
  struct fid_av* av;
  fi_addr_t* fi_addr;
  uintptr_t* heap_addr;

  void* heap;
  uint32_t heap_rkey;
  int recv_posted;
  lch* mv;
} ofi_server __attribute__((aligned(64)));

LC_INLINE void ofi_init(lch* mv, size_t heap_size, ofi_server** s_ptr);
LC_INLINE void ofi_post_recv(ofi_server* s, lc_packet* p);
LC_INLINE int ofi_write_send(ofi_server* s, int rank, void* buf, size_t size,
                             lc_packet* ctx, uint32_t proto);
LC_INLINE void ofi_write_rma(ofi_server* s, int rank, void* from,
                             uintptr_t addr, uint32_t rkey, size_t size,
                             lc_packet* ctx, uint32_t proto);

LC_INLINE void ofi_write_rma_signal(ofi_server* s, int rank, void* buf,
                                    uintptr_t addr, uint32_t rkey, size_t size,
                                    uint32_t sid, lc_packet* ctx,
                                    uint32_t proto);

LC_INLINE void ofi_finalize(ofi_server* s);

static uint32_t next_key = 1;

LC_INLINE uintptr_t _real_ofi_reg(ofi_server* s, void* buf, size_t size)
{
  struct fid_mr* mr;
  FI_SAFECALL(fi_mr_reg(s->domain, buf, size,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, next_key++, 0,
                        &mr, 0));
  return (uintptr_t)mr;
}

LC_INLINE uintptr_t ofi_rma_reg(ofi_server* s, void* buf, size_t size)
{
  return (uintptr_t)dreg_register(s, buf, size);
  // return _real_ofi_reg(s, buf, size);
}

LC_INLINE int ofi_rma_dereg(uintptr_t mem)
{
  dreg_unregister((dreg_entry*)mem);
  return 1;
  // return fi_close((struct fid*) mem);
}

LC_INLINE uint32_t ofi_rma_key(uintptr_t mem)
{
  // return fi_mr_key((struct fid_mr*) mem);
  return fi_mr_key((struct fid_mr*)(((dreg_entry*)mem)->memhandle[0]));
}

LC_INLINE void ofi_init(lch* mv, size_t heap_size, ofi_server** s_ptr)
{
  // Create hint.
  struct fi_info* hints;
  hints = fi_allocinfo();
  hints->ep_attr->type = FI_EP_RDM;
  hints->domain_attr->mr_mode = FI_MR_BASIC;
  hints->caps = FI_RMA | FI_MSG;
  hints->mode = FI_CONTEXT | FI_LOCAL_MR;

  ofi_server* s = malloc(sizeof(ofi_server));

  // Create info.
  FI_SAFECALL(fi_getinfo(FI_VERSION(1, 0), NULL, NULL, 0, hints, &s->fi));

  // Create libfabric obj.
  FI_SAFECALL(fi_fabric(s->fi->fabric_attr, &s->fabric, NULL));

  // Create domain.
  FI_SAFECALL(fi_domain(s->fabric, s->fi, &s->domain, NULL));

  // Create end-point;
  FI_SAFECALL(fi_endpoint(s->domain, s->fi, &s->ep, NULL));

  // Create cq.
  struct fi_cq_attr* cq_attr = malloc(sizeof(struct fi_cq_attr));
  memset(cq_attr, 0, sizeof(struct fi_cq_attr));
  cq_attr->format = FI_CQ_FORMAT_DATA;
  cq_attr->size = MAX_CQ_SIZE;
  // FI_SAFECALL(fi_cq_open(s->domain, cq_attr, &s->scq, NULL));
  FI_SAFECALL(fi_cq_open(s->domain, cq_attr, &s->rcq, NULL));

  // Bind my ep to cq.
  // FI_SAFECALL(
  //    fi_ep_bind(s->ep, (fid_t)s->scq, FI_SEND | FI_TRANSMIT));
  FI_SAFECALL(
      fi_ep_bind(s->ep, (fid_t)s->rcq, FI_SEND | FI_TRANSMIT | FI_RECV));

  dreg_init();

  // Get memory for heap.
  s->heap = 0;  // std::move(unique_ptr<char[]>(new char[heap_size]));
  posix_memalign(&s->heap, lcg_page_size, heap_size);

  FI_SAFECALL(fi_mr_reg(s->domain, s->heap, heap_size,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, 0, 0,
                        &s->mr_heap, 0));

  s->heap_rkey = fi_mr_key(s->mr_heap);
  MPI_Comm_rank(MPI_COMM_WORLD, &mv->me);
  MPI_Comm_size(MPI_COMM_WORLD, &mv->size);

  // Now exchange end-point address and heap address.
  size_t addrlen = 0;
  fi_getname((fid_t)s->ep, NULL, &addrlen);
  void* addr = malloc(addrlen + sizeof(uintptr_t));
  FI_SAFECALL(fi_getname((fid_t)s->ep, addr, &addrlen));

  // Set heap address at the end. TODO(danghvu): Need a more generic way.
  uintptr_t myaddr = (uintptr_t)s->heap;
  *(uintptr_t*)((char*)addr + addrlen) = myaddr;

  s->fi_addr = malloc(sizeof(fi_addr_t) * mv->size);
  s->heap_addr = malloc(sizeof(uintptr_t) * mv->size);

  struct fi_av_attr av_attr;
  memset(&av_attr, 0, sizeof(av_attr));
  av_attr.type = FI_AV_MAP;
  FI_SAFECALL(fi_av_open(s->domain, &av_attr, &s->av, NULL));
  FI_SAFECALL(fi_ep_bind(s->ep, (fid_t)s->av, 0));
  FI_SAFECALL(fi_enable(s->ep));

  for (int i = 0; i < mv->size; i++) {
    if (i != mv->me) {
      void* destaddr = malloc(addrlen + sizeof(uintptr_t));
      MPI_Sendrecv(addr, addrlen + sizeof(uintptr_t), MPI_BYTE, i, 99, destaddr,
                   addrlen + sizeof(uintptr_t), MPI_BYTE, i, 99, MPI_COMM_WORLD,
                   MPI_STATUS_IGNORE);
      if (fi_av_insert(s->av, destaddr, 1, &s->fi_addr[i], 0, NULL) == -1) {
        MPI_Abort(MPI_COMM_WORLD, -1);
      }
      s->heap_addr[i] = *(uintptr_t*)((char*)destaddr + addrlen);
    }
  }

  s->recv_posted = 0;
  s->mv = mv;
  *s_ptr = s;
}

extern double lc_ptime;

LC_INLINE int ofi_progress_once(ofi_server* s)
{
  struct fi_cq_data_entry entry;
  int ret = fi_cq_read(s->rcq, &entry, 1);
  if (ret > 0) {
    return 1;
  }
  return 0;
}

LC_INLINE int ofi_progress_send(ofi_server* s)
{
  struct fi_cq_data_entry entry[MAX_POLL];
  struct fi_cq_err_entry error;
  ssize_t ret;
  int rett = 0;

  do {
    ret = fi_cq_read(s->scq, &entry, MAX_POLL);
    // t1 += MPI_Wtime();
    if (ret > 0) {
      // Got an entry here ?
      for (int i = 0; i < ret; i++) {
        lc_packet* p = (lc_packet*)entry[i].op_context;
        lc_serve_send(s->mv, p, p->context.proto);
      }
      rett = 1;
#ifdef SERVER_FI_DEBUG
    } else if (ret == -FI_EAGAIN) {
    } else {
      fi_cq_readerr(s->rcq, &error, 0);
      printf("Err: %s\n", fi_strerror(error.err));
      MPI_Abort(MPI_COMM_WORLD, error.err);
#endif
    }
  } while (ret > 0);

  if (s->recv_posted < MAX_RECV)
    ofi_post_recv(s, lc_pool_get_nb(s->mv->pkpool));

#ifdef SERVER_FI_DEBUG
  if (s->recv_posted == 0) {
    fprintf(stderr, "WARNING DEADLOCK\n");
  }
#endif

  return rett;
}

LC_INLINE int ofi_progress(ofi_server* s)
{
  // double t1 = -(MPI_Wtime());
  struct fi_cq_data_entry entry[MAX_POLL];
  struct fi_cq_err_entry error;
  ssize_t ret;
  int rett = 0;

  do {
    ret = fi_cq_read(s->rcq, &entry, MAX_POLL);
    // t1 += MPI_Wtime();
    if (ret > 0) {
      // Got an entry here ?
      for (int i = 0; i < ret; i++) {
        if (entry[i].flags & FI_RECV) {
          s->recv_posted--;
          lc_serve_recv(s->mv, (lc_packet*)entry[i].op_context, entry[i].data);
        } else if (entry[i].flags & FI_REMOTE_CQ_DATA) {
          // NOTE(danghvu): In OFI, a imm data is transferred without
          // comsuming a posted receive.
          lc_serve_imm(s->mv, entry[i].data);
        } else if (entry[i].flags & FI_RECV) {
          s->recv_posted--;
          lc_serve_recv(s->mv, (lc_packet*)entry[i].op_context, entry[i].data);
        } else {
          lc_packet* p = (lc_packet*)entry[i].op_context;
          lc_serve_send(s->mv, p, p->context.proto);
        }
      }
      rett = 1;
#ifdef SERVER_FI_DEBUG
    } else if (ret == -FI_EAGAIN) {
    } else {
      fi_cq_readerr(s->rcq, &error, 0);
      printf("Err: %s\n", fi_strerror(error.err));
      MPI_Abort(MPI_COMM_WORLD, error.err);
#endif
    }
  } while (ret > 0);

  if (s->recv_posted < MAX_RECV)
    ofi_post_recv(s, lc_pool_get_nb(s->mv->pkpool));

#ifdef SERVER_FI_DEBUG
  if (s->recv_posted == 0) {
    fprintf(stderr, "WARNING DEADLOCK\n");
  }
#endif

  return rett;
}

LC_INLINE void ofi_post_recv(ofi_server* s, lc_packet* p)
{
  if (p == NULL) return;
  FI_SAFECALL(
      fi_recv(s->ep, &p->data, POST_MSG_SIZE, 0, FI_ADDR_UNSPEC, &p->context));
  s->recv_posted++;
}

LC_INLINE int ofi_write_send(ofi_server* s, int rank, void* buf, size_t size,
                             lc_packet* ctx, uint32_t proto)
{
  if (size <= SERVER_MAX_INLINE) {
    FI_SAFECALL(fi_injectdata(s->ep, buf, size, proto, s->fi_addr[rank]));
    lc_serve_send(s->mv, ctx, proto);
    return 0;
  } else {
    ctx->context.proto = proto;
    FI_SAFECALL(fi_senddata(s->ep, buf, size, 0, proto, s->fi_addr[rank],
                            (struct fi_context*)ctx));
    return 1;
  }
}

LC_INLINE void ofi_write_rma(ofi_server* s, int rank, void* from,
                             uintptr_t addr, uint32_t rkey, size_t size,
                             lc_packet* ctx, uint32_t proto)
{
  ctx->context.proto = proto;
  FI_SAFECALL(fi_write(s->ep, from, size, 0, s->fi_addr[rank], 0, rkey, ctx));
}

LC_INLINE void ofi_write_rma_signal(ofi_server* s, int rank, void* buf,
                                    uintptr_t addr, uint32_t rkey, size_t size,
                                    uint32_t sid, lc_packet* ctx,
                                    uint32_t proto)
{
  ctx->context.proto = proto;
  FI_SAFECALL(fi_writedata(s->ep, buf, size, 0, sid, s->fi_addr[rank], addr,
                           rkey, ctx));
}

LC_INLINE void ofi_finalize(ofi_server* s) { free(s); }
LC_INLINE uint32_t ofi_heap_rkey(ofi_server* s, int node __UNUSED__)
{
  return s->heap_rkey;
}

LC_INLINE void* ofi_heap_ptr(ofi_server* s) { return s->heap; }
#define lc_server_init ofi_init
#define lc_server_send ofi_write_send
#define lc_server_rma ofi_write_rma
#define lc_server_rma_signal ofi_write_rma_signal
#define lc_server_heap_rkey ofi_heap_rkey
#define lc_server_heap_ptr ofi_heap_ptr
#define lc_server_progress ofi_progress
#define lc_server_finalize ofi_finalize
#define lc_server_post_recv ofi_post_recv
#define lc_server_progress_once ofi_progress_once

#define lc_server_rma_reg ofi_rma_reg
#define lc_server_rma_key ofi_rma_key
#define lc_server_rma_dereg ofi_rma_dereg

#endif
