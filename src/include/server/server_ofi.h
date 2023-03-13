#ifndef SERVER_OFI_H_
#define SERVER_OFI_H_

#include "config.h"

#include <rdma/fabric.h>
#include <rdma/fi_domain.h>
#include <rdma/fi_endpoint.h>
#include <rdma/fi_tagged.h>
#include <stdlib.h>
#include <string.h>

#include <rdma/fi_cm.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_rma.h>

#include "config.h"
#include "macro.h"
#include "pm.h"
#include "dreg.h"

#define SERVER_FI_DEBUG

#ifdef SERVER_FI_DEBUG
#define FI_SAFECALL(x)                                                    \
  {                                                                       \
    int err = (x);                                                        \
    if (err) {                                                            \
      printf("err : %s (%s:%d)\n", fi_strerror(err), __FILE__, __LINE__); \
      exit(err);                                     \
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

#define ALIGNMENT (4096)
#define ALIGNEDX(x) \
  (void*)((((uintptr_t)x + ALIGNMENT - 1) / ALIGNMENT * ALIGNMENT))
#define MAX_CQ_SIZE (16 * 1024)
#define MAX_POLL 8

typedef struct lc_server {
  SERVER_COMMON

  struct fi_info* fi;
  struct fid_fabric* fabric;
  struct fid_domain* domain;
  struct fid_ep* ep;
  struct fid_cq* scq;
  struct fid_cq* rcq;
  struct fid_mr* mr_heap;
  struct fid_av* av;
  fi_addr_t* fi_addr;

  uint32_t heap_rkey;
} lc_server __attribute__((aligned(64)));

static inline uintptr_t _real_ofi_reg(lc_server* s, void* buf, size_t size)
{
  struct fid_mr* mr;
  FI_SAFECALL(fi_mr_reg(s->domain, buf, size,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, lc_next_rdma_key++, 0,
                        &mr, 0));
  return (uintptr_t)mr;
}

static inline uintptr_t lc_server_rma_reg (lc_server* s, void* buf, size_t size)
{
  return (uintptr_t)dreg_register(s, buf, size);
  // return _real_ofi_reg(s, buf, size);
}

static inline void lc_server_rma_dereg(uintptr_t mem)
{
  dreg_unregister((dreg_entry*)mem);
}

static inline uint32_t lc_server_rma_key(uintptr_t mem)
{
  return fi_mr_key((struct fid_mr*)(((dreg_entry*)mem)->memhandle[0]));
}

static inline void lc_server_init(int id, lc_server** dev)
{
  lc_server* s = NULL;
  posix_memalign((void**)&s, 8192, sizeof(struct lc_server));
  *dev = s;

  // Create hint.
  struct fi_info* hints;
  hints = fi_allocinfo();
  hints->ep_attr->type = FI_EP_RDM;
  hints->domain_attr->mr_mode = FI_MR_BASIC;
  hints->caps = FI_RMA | FI_TAGGED;
  hints->mode = FI_CONTEXT | FI_LOCAL_MR;

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
  cq_attr->format = FI_CQ_FORMAT_TAGGED;
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
  s->heap_addr = 0; 
  posix_memalign(&s->heap_addr, 4096, LCI_REGISTERED_MEMORY_SIZE);

  FI_SAFECALL(fi_mr_reg(s->domain, s->heap_addr, LCI_REGISTERED_MEMORY_SIZE,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, 0, 0,
                        &s->mr_heap, 0));

  s->heap_rkey = fi_mr_key(s->mr_heap);
  s->id = id;

  struct fi_av_attr av_attr;
  memset(&av_attr, 0, sizeof(av_attr));
  av_attr.type = FI_AV_MAP;
  FI_SAFECALL(fi_av_open(s->domain, &av_attr, &s->av, NULL));
  FI_SAFECALL(fi_ep_bind(s->ep, (fid_t)s->av, 0));
  FI_SAFECALL(fi_enable(s->ep));

  // Now exchange end-point address and heap address.
  size_t addrlen = 0;
  fi_getname((fid_t)s->ep, NULL, &addrlen);
  void* addr = malloc(addrlen + sizeof(uintptr_t));
  FI_SAFECALL(fi_getname((fid_t)s->ep, addr, &addrlen));

  // Set heap address at the end. TODO(danghvu): Need a more generic way.
  uintptr_t myaddr = (uintptr_t)s->heap_addr;
  *(uintptr_t*)((char*)addr + addrlen) = myaddr;

  s->fi_addr = malloc(sizeof(fi_addr_t) * LCI_NUM_PROCESSES);

  posix_memalign((void**)&(s->rep), LC_CACHE_LINE,
                 sizeof(struct lc_rep) * LCI_NUM_PROCESSES);
  char tmp[256];
//  printf("%d\n", addrlen);
  sprintf(tmp, "%llu-%llu-%llu", ((unsigned long long*)addr)[0],
      ((unsigned long long*) addr)[1], (unsigned long long) myaddr);
  free(addr);
  lc_pm_publish(LCI_RANK, id, tmp);

  for (int i = 0; i < LCI_NUM_PROCESSES; i++) {
    if (i != LCI_RANK) {
      struct lc_rep* rep = &s->rep[i];
      lc_pm_getname(i, id, tmp);
      uintptr_t destaddr[2];
      uintptr_t heapaddr;
      sscanf(tmp, "%llu-%llu-%llu",
          (unsigned long long*)&destaddr[0], (unsigned long long*) &destaddr[1],
          (unsigned long long*)&heapaddr);
      if (fi_av_insert(s->av, &destaddr[0], 1, &s->fi_addr[i], 0, NULL) == -1) {
        exit(-1);
      }
      rep->rank = i;
      rep->handle = (void*)s->fi_addr[i];
      rep->base = (uintptr_t)heapaddr;
    }
  }

  s->recv_posted = 0;
}

static inline void lc_server_post_recv(lc_server* s, lc_packet* p);

static inline int lc_server_progress(lc_server* s)
{
  // double t1 = -(MPI_Wtime());
  struct fi_cq_tagged_entry entry[MAX_POLL];
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
          lc_packet* p = (lc_packet*) entry[i].op_context;
          p->context.sync = &p->context.sync_s;
          int rank = p->context.sync->request.rank = (entry[i].tag);
          p->context.sync->request.__reserved__ = s->fi_addr[rank];
          p->context.sync->request.length = entry[i].len;
          lc_serve_recv(p, entry[i].data);
        } else if (entry[i].flags & FI_REMOTE_CQ_DATA) {
          // NOTE(danghvu): In OFI, a imm data is transferred without
          // comsuming a posted receive.
          lc_serve_imm(entry[i].data);
        } else {
          lc_packet* p = (lc_packet*)entry[i].op_context;
          lc_serve_send(p);
        }
      }
      rett = 1;
#ifdef SERVER_FI_DEBUG
    } else if (ret == -FI_EAGAIN) {
    } else {
      fi_cq_readerr(s->rcq, &error, 0);
      printf("Err: %s\n", fi_strerror(error.err));
      exit(error.err);
#endif
    }
  } while (ret > 0);

  if (s->recv_posted < LC_SERVER_MAX_RCVS)
    lc_server_post_recv(s, lc_pool_get_nb(s->pkpool));

#ifdef SERVER_FI_DEBUG
  if (s->recv_posted == 0) {
    fprintf(stderr, "WARNING DEADLOCK\n");
  }
#endif

  return rett;
}

static inline void lc_server_post_recv(lc_server* s, lc_packet* p)
{
  if (p == NULL) return;
  FI_SAFECALL(
      fi_trecv(s->ep, &p->data, SHORT_MSG_SIZE, 0, FI_ADDR_UNSPEC, /*any*/0, ~0 /*ignore*/, &p->context));
  s->recv_posted++;
}

/*
static inline void ofi_write_rma(lc_server* s, int rank, void* from,
                             uintptr_t addr, uint32_t rkey, size_t size,
                             lc_packet* ctx, uint32_t proto)
{
  ctx->context.proto = proto;
  FI_SAFECALL(fi_write(s->ep, from, size, 0, s->fi_addr[rank], 0, rkey, ctx));
}

static inline void ofi_write_rma_signal(lc_server* s, int rank, void* buf,
                                    uintptr_t addr, uint32_t rkey, size_t size,
                                    uint32_t sid, lc_packet* ctx,
                                    uint32_t proto)
{
  ctx->context.proto = proto;
  FI_SAFECALL(fi_writedata(s->ep, buf, size, 0, sid, s->fi_addr[rank], addr,
                           rkey, ctx));
}*/

static inline void lc_server_sendm(lc_server* s, void* rep, size_t size,
                                   lc_packet* ctx, uint32_t proto)
{
  ctx->context.proto = proto;
  FI_SAFECALL(fi_tsenddata(s->ep, ctx->data.buffer, size, 0, proto, rep,
        LCI_RANK, (struct fi_context*)ctx));
}

static inline void lc_server_sends(lc_server* s, void* rep, void* ubuf,
                                   size_t size, uint32_t proto)
{
  FI_SAFECALL(fi_tinjectdata(s->ep, ubuf, size, proto, rep, LCI_RANK));
}

static inline void lc_server_puts(lc_server* s, void* rep, void* buf,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey __UNUSED__, uint32_t meta,
                                   size_t size)
{
}

static inline void lc_server_putm(lc_server* s, void* rep,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey __UNUSED__, size_t size,
                                   uint32_t meta, lc_packet* ctx)
{
}

static inline void lc_server_putl(lc_server* s, void* rep, void* buffer,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey, size_t size, uint32_t sid,
                                   lc_packet* ctx)
{
}

static inline void lc_server_rma_rtr(lc_server* s, void* rep, void* buf,
                                     uintptr_t addr __UNUSED__, uint32_t rkey,
                                     size_t size, uint32_t sid, lc_packet* ctx)
{
}

static inline void lc_server_finalize(lc_server* s) { free(s); }

static inline void* lc_server_heap_ptr(lc_server* s) { return (void*) s->heap_addr; }

#endif
