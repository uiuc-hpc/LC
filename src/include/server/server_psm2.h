#ifndef SERVER_PSM_H_
#define SERVER_PSM_H_

#include "config.h"

#include <psm2.h>    /* required for core PSM2 functions */
#include <psm2_mq.h> /* required for PSM2 MQ functions (send, recv, etc) */
#include <psm2_am.h>

#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "pm.h"
#include "macro.h"
#include "lcrq.h"

#ifdef LC_SERVER_DEBUG
#define PSM_SAFECALL(x)                                       \
  {                                                           \
    int err = (x);                                            \
    if (err != PSM2_OK && err != PSM2_MQ_NO_COMPLETIONS) {    \
      fprintf(stderr, "err : (%s:%d)\n", __FILE__, __LINE__); \
      exit(err);                                              \
    }                                                         \
  }                                                           \
  while (0)                                                   \
    ;

#else
#define PSM_SAFECALL(x) \
  {                     \
    (x);                \
  }
#endif

#define ALIGNMENT (8192)
#define ALIGNEDX(x) \
  (void*)((((uintptr_t)x + ALIGNMENT - 1) / ALIGNMENT * ALIGNMENT))
#define MAX_CQ_SIZE (16 * 1024)
#define MAX_POLL 8

#define PSM_RECV_DATA ((uint32_t)(0x1 << 28))
#define PSM_RECV_RDMA ((uint32_t)(0x2 << 28))
#define PSM_RECV_RDMA_SID ((uint32_t)(0x3 << 28))
#define PSM_RECV_RDMA_RTP ((uint32_t)(0x4 << 28))

#define PSM_RDMA_IMM ((uint32_t)1 << 31)

struct lc_server;

struct psm_mr {
  struct lc_server* server;
  uintptr_t addr;
  size_t size;
  uint32_t rkey;
  psm2_mq_req_t req;
};

typedef struct lc_server {
  SERVER_COMMON

  psm2_uuid_t uuid;

  // Endpoint + Endpoint ID.
  psm2_ep_t myep;
  psm2_epid_t myepid;
  psm2_mq_t mq;

  uint32_t heap_rkey;
  lcrq_t free_mr;
} lc_server __attribute__((aligned(LC_CACHE_LINE)));

static psm2_mq_tag_t LCI_PSM2_TAGSEL = {{0x0, 0x0, 0x0FFFFFFF}};
static inline void lc_server_post_recv(lc_server* s, lc_packet* p);

#define PSM_RECV ((uint64_t)1 << 62)
#define PSM_SEND ((uint64_t)1 << 63)

// Buffered send-recv.
#define PSM_TAG_TRECV_DATA()                        \
  {                                                 \
    .tag0 = 0x0, .tag1 = 0x0, .tag2 = PSM_RECV_DATA \
  }
#define PSM_TAG_TSEND_DATA(proto, rank)                \
  {                                                    \
    .tag0 = proto, .tag1 = rank, .tag2 = PSM_RECV_DATA \
  }

// Buffered RDMA.
#define PSM_TAG_SRDMA(offset)                          \
  {                                                    \
    .tag0 = offset, .tag1 = 0x0, .tag2 = PSM_RECV_RDMA \
  }
#define PSM_TAG_SRDMA_META(offset, meta)                    \
  {                                                         \
    .tag0 = offset, .tag1 = meta, .tag2 = PSM_RECV_RDMA_SID \
  }

// Non-buffered RDMA.
#define PSM_TAG_SRDMA_RTS(offset, rkey)                     \
  {                                                         \
    .tag0 = offset, .tag1 = rkey, .tag2 = PSM_RECV_RDMA_RTP \
  }
#define PSM_TAG_SRDMA_DAT(rkey)        \
  {                                    \
    .tag0 = 0, .tag1 = 0, .tag2 = rkey \
  }
#define PSM_TAG_SRDMA_DAT_META(rkey, meta) \
  {                                        \
    .tag0 = 1, .tag1 = meta, .tag2 = rkey  \
  }
#define PSM_TAG_SRDMA_DAT_IMM(rkey, sid)            \
  {                                                 \
    .tag0 = sid, .tag1 = PSM_RDMA_IMM, .tag2 = rkey \
  }

// RDMA RECV
#define PSM_TAG_RRDMA_DAT(rkey)        \
  {                                    \
    .tag0 = 0, .tag1 = 0, .tag2 = rkey \
  }

static inline void lc_server_post_recv_rma(lc_server* s, void* addr,
                                           size_t size, uint32_t rkey,
                                           uintptr_t ctx, psm2_mq_req_t* req)
{
  psm2_mq_tag_t rtag = PSM_TAG_RRDMA_DAT(rkey);

  PSM_SAFECALL(psm2_mq_irecv2(s->mq, PSM2_MQ_ANY_ADDR, &rtag, /* message tag */
                              &LCI_PSM2_TAGSEL, /* message tag mask */
                              0,                /* no flags */
                              addr, size, (void*)(PSM_RECV | ctx), req));
}

static inline struct psm_mr* lc_server_get_free_mr(lc_server* s)
{
  struct psm_mr* mr = (struct psm_mr*)lcrq_dequeue(&s->free_mr);
  if (!mr) {
    posix_memalign((void**)&mr, LC_CACHE_LINE, sizeof(struct psm_mr));
  }
  return mr;
}

static inline uint32_t lc_server_get_free_key()
{
  return (__sync_fetch_and_add(&lc_next_rdma_key, 1) & 0x0fffffff);
}

static inline uintptr_t _real_psm_reg(lc_server* s, void* buf, size_t size)
{
  struct psm_mr* mr = lc_server_get_free_mr(s);
  mr->server = s;
  mr->addr = (uintptr_t)buf;
  mr->size = size;
  mr->rkey = lc_server_get_free_key();
  lc_server_post_recv_rma(s, buf, size, mr->rkey, 0, &(mr->req));
  return (uintptr_t)mr;
}

static inline int _real_psm_free(uintptr_t mem)
{
  struct psm_mr* mr = (struct psm_mr*)mem;
  lcrq_enqueue(&mr->server->free_mr, (void*)mr);
  return 1;
}

static inline uintptr_t lc_server_rma_reg(lc_server* s, void* buf, size_t size)
{
  return _real_psm_reg(s, buf, size);
}

static inline void lc_server_rma_dereg(uintptr_t mem) { _real_psm_free(mem); }

static inline uint32_t lc_server_rma_key(uintptr_t mem)
{
  return ((struct psm_mr*)mem)->rkey;
}

static inline void lc_server_init(int id, lc_server** dev)
{
  // setenv("I_MPI_FABRICS", "ofa", 1);
  // setenv("PSM2_SHAREDCONTEXTS", "0", 1);
  // setenv("PSM2_RCVTHREAD", "0", 1);
  lc_server* s = NULL;
  posix_memalign((void**)&s, 8192, sizeof(struct lc_server));
  *dev = s;

  int ver_major = PSM2_VERNO_MAJOR;
  int ver_minor = PSM2_VERNO_MINOR;
  s->id = id;

  static int psm2_initialized = 0;
  if (!psm2_initialized) {
    PSM_SAFECALL(psm2_init(&ver_major, &ver_minor));
    psm2_initialized = 1;
  }

  /* Setup the endpoint options struct */
  struct psm2_ep_open_opts option;
  PSM_SAFECALL(psm2_ep_open_opts_get_defaults(&option));
  option.affinity = 0;  // Disable affinity.

  psm2_uuid_generate(s->uuid);

  /* Attempt to open a PSM2 endpoint. This allocates hardware resources. */
  PSM_SAFECALL(psm2_ep_open(s->uuid, &option, &s->myep, &s->myepid));

  LCI_RANK = LCI_RANK;
  s->recv_posted = 0;

  posix_memalign(
      (void**) &s->heap_addr, 8192,
      LC_SERVER_NUM_PKTS * LC_PACKET_SIZE * 2 + LCI_REGISTERED_MEMORY_SIZE);

  lcrq_init(&s->free_mr);
  for (int i = 0; i < 256; i++)
    lcrq_enqueue(&s->free_mr, malloc(sizeof(struct psm_mr)));

  PSM_SAFECALL(psm2_mq_init(s->myep, PSM2_MQ_ORDERMASK_ALL, NULL, 0, &s->mq));

  char ep_name[256];
  s->heap_rkey = lc_server_get_free_key();
  sprintf(ep_name, "%llu-%llu-%d", (unsigned long long)s->myepid,
          (unsigned long long)s->heap_addr, (uint32_t)s->heap_rkey);
  lc_pm_publish(LCI_RANK, id, ep_name);

  posix_memalign((void**)&(s->rep), LC_CACHE_LINE,
                 sizeof(struct lc_rep) * LCI_NUM_PROCESSES);

  for (int i = 0; i < LCI_NUM_PROCESSES; i++) {
    if (i != LCI_RANK) {
      struct lc_rep* rep = &s->rep[i];
      psm2_error_t errs;
      lc_pm_getname(i, id, ep_name);
      psm2_epid_t destaddr;
      psm2_epaddr_t epaddr;
      unsigned long long baseaddr;
      uint32_t rkey;
      sscanf(ep_name, "%llu-%llu-%d", (unsigned long long*)&destaddr,
             (unsigned long long*)&baseaddr, &rkey);
      PSM_SAFECALL(
          psm2_ep_connect(s->myep, 1, &destaddr, NULL, &errs, &epaddr, 0));
      rep->rank = i;
      rep->handle = (void*)epaddr;
      rep->base = (uintptr_t)baseaddr;
      rep->rkey = rkey;
    }
  }
}

static inline int lc_server_progress(lc_server* s)
{
  psm2_mq_req_t req;
  psm2_mq_status2_t status;
  psm2_error_t err = psm2_mq_ipeek2(s->mq, &req, &status);

  if (err == PSM2_OK) {
#ifndef USE_MINI_PSM2
    psm2_mq_test2(&req, &status);  // we need the status
#endif
    uintptr_t ctx = (uintptr_t)status.context;
    if (ctx & PSM_RECV) {
      lc_packet* p = (lc_packet*)(ctx ^ PSM_RECV);
      uint32_t pk_type = (status.msg_tag.tag2 & 0xf0000000);
      // Simple recv.
      if (pk_type == PSM_RECV_DATA) {
        p->context.sync = &p->context.sync_s;
        p->context.sync->request.__reserved__ = status.msg_peer;
        p->context.sync->request.rank = status.msg_tag.tag1;
        p->context.sync->request.length = (status.msg_length);
        uint32_t proto = status.msg_tag.tag0;
        lc_serve_recv(p, proto);
        s->recv_posted--;
      } else if (pk_type == PSM_RECV_RDMA) {
        p->context.sync = &p->context.sync_s;
        uintptr_t addr = (uintptr_t)status.msg_tag.tag0 + (uintptr_t)s->heap_addr;
        memcpy((void*)addr, p->data.buffer, status.msg_length);
        lc_server_post_recv(s, p);
      } else if (pk_type == PSM_RECV_RDMA_SID) {
        p->context.sync = &p->context.sync_s;
        uintptr_t addr = (uintptr_t)status.msg_tag.tag0 + (uintptr_t)s->heap_addr;
        memcpy((void*)addr, p->data.buffer, status.msg_length);
        lc_serve_recv_rdma(p, status.msg_tag.tag1);
        s->recv_posted--;
      } else if (pk_type == PSM_RECV_RDMA_RTP) {
        uintptr_t addr = (uintptr_t)status.msg_tag.tag0 + (uintptr_t)s->heap_addr;
        uint32_t rkey = status.msg_tag.tag1;
        size_t size;
        memcpy(&size, p->data.buffer, sizeof(size_t));
        lc_server_post_recv_rma(s, (void*)addr, size, rkey, (uintptr_t)p,
                                (psm2_mq_req_t*)p);
        s->recv_posted--;
      } else if (status.msg_tag.tag1 == PSM_RDMA_IMM) {
        uint32_t off = status.msg_tag.tag0;
        lc_packet* p = (lc_packet*)(s->heap_addr + off);
        lc_serve_imm(p);
      } else if (status.msg_tag.tag0) {
        p->context.sync = &p->context.sync_s;
        lc_serve_recv_rdma(p, status.msg_tag.tag1);
      } else {
        p->context.sync = &p->context.sync_s;
        lc_pool_put(s->pkpool, p);
      }
    } else if (ctx & PSM_SEND) {
      lc_packet* p = (lc_packet*)(ctx ^ PSM_SEND);
      if (p) lc_serve_send(p);
    }
  }

  // Post receive if any.
  if (s->recv_posted < LC_SERVER_MAX_RCVS)
    lc_server_post_recv(s, lc_pool_get_nb(s->pkpool));

  return (err == PSM2_OK);
}

static inline void lc_server_post_recv(lc_server* s, lc_packet* p)
{
  if (p == NULL) {
    if (s->recv_posted == LC_SERVER_MAX_RCVS / 2 && !lcg_deadlock) {
      lcg_deadlock = 1;
      dprintf("WARNING-LC: deadlock alert\n");
    }
    return;
  }

  psm2_mq_tag_t rtag = PSM_TAG_TRECV_DATA();

  PSM_SAFECALL(psm2_mq_irecv2(s->mq, PSM2_MQ_ANY_ADDR, &rtag, /* message tag */
                              &LCI_PSM2_TAGSEL, /* message tag mask */
                              0,                /* no flags */
                              &p->data, SHORT_MSG_SIZE,
                              (void*)(PSM_RECV | (uintptr_t)&p->context),
                              (psm2_mq_req_t*)p));

  if (++s->recv_posted == LC_SERVER_MAX_RCVS && lcg_deadlock) lcg_deadlock = 0;
}

static inline void lc_server_sendm(lc_server* s, void* rep, size_t size,
                                   lc_packet* ctx, uint32_t proto)
{
  psm2_mq_tag_t rtag = PSM_TAG_TSEND_DATA(proto, LCI_RANK);

  PSM_SAFECALL(psm2_mq_isend2(s->mq, rep, 0, /* no flags */
                              &rtag, ctx->data.buffer, size,
                              (void*)(PSM_SEND | (uintptr_t)ctx),
                              (psm2_mq_req_t*)ctx));
}

static inline void lc_server_sends(lc_server* s, void* rep, void* ubuf,
                                   size_t size, uint32_t proto)
{
  psm2_mq_tag_t rtag = PSM_TAG_TSEND_DATA(proto, LCI_RANK);
  PSM_SAFECALL(psm2_mq_send2(s->mq, rep, 0, &rtag, ubuf, size));
}

static inline void lc_server_puts(lc_server* s, void* rep, void* buf,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey __UNUSED__, uint32_t meta,
                                   size_t size)
{
  psm2_mq_tag_t rtag = PSM_TAG_SRDMA_META(offset, meta);
  PSM_SAFECALL(psm2_mq_send2(s->mq, rep, 0, &rtag, buf, size));
}

static inline void lc_server_putm(lc_server* s, void* rep,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey __UNUSED__, size_t size,
                                   uint32_t meta, lc_packet* ctx)
{
  psm2_mq_tag_t rtag = PSM_TAG_SRDMA_META(offset, meta);
  PSM_SAFECALL(psm2_mq_isend2(s->mq, rep, 0, &rtag, ctx->data.buffer, size,
                              (void*)(PSM_SEND | (uintptr_t)ctx),
                              (psm2_mq_req_t*)ctx));
}

static inline void lc_server_putl(lc_server* s, void* rep, void* buffer,
                                   uintptr_t base __UNUSED__, uint32_t offset,
                                   uint32_t rkey, size_t size, uint32_t sid,
                                   lc_packet* ctx)
{
  psm2_mq_tag_t rtag = PSM_TAG_SRDMA_RTS(offset, rkey);
  PSM_SAFECALL(psm2_mq_send2(s->mq, rep, 0, &rtag, &size, sizeof(size_t)));

  psm2_mq_tag_t rtag2 = PSM_TAG_SRDMA_DAT_META(rkey, sid);
  PSM_SAFECALL(psm2_mq_isend2(s->mq, rep, 0, &rtag2, buffer, size,
                              (void*)(PSM_SEND | (uintptr_t)ctx),
                              (psm2_mq_req_t*)ctx));
}

#if 0
static inline void lc_server_get(lc_server* s, int rank, void* buf,
                       uintptr_t addr, size_t offset, uint32_t rkey __UNUSED__, size_t size,
                       uint32_t sid, lc_packet* ctx __UNUSED__) // FIXME
{
  psm2_mq_tag_t rtag;
  rtag.tag0 = sid;
  rtag.tag1 = 0;
  rtag.tag2 = PSM_CTRL_MSG;

  struct psm_mr* mr = (struct psm_mr*)  lc_server_rma_reg(s, buf, size);
  struct psm_ctrl_msg msg = {REQ_TO_PUT, s->ep->me, addr + offset, size, mr->rkey, sid};

  PSM_SAFECALL(psm2_mq_send2(s->mq, s->epaddr[rank], 0,    /* no flags */
               &rtag, /* tag */
               &msg, sizeof(struct psm_ctrl_msg)));
}
#endif

static inline void lc_server_rma_rtr(lc_server* s, void* rep, void* buf,
                                     uintptr_t addr __UNUSED__, uint32_t rkey,
                                     size_t size, uint32_t sid, lc_packet* ctx)
{
  psm2_mq_tag_t rtag = PSM_TAG_SRDMA_DAT_IMM(rkey, sid);

  PSM_SAFECALL(psm2_mq_isend2(s->mq, rep, 0, /* no flags */
                              &rtag, buf, size,
                              (void*)(PSM_SEND | (uintptr_t)ctx),
                              (psm2_mq_req_t*)ctx));
}

static inline void lc_server_finalize(lc_server* s) { free(s); }

static inline void* lc_server_heap_ptr(lc_server* s) { return (void*) s->heap_addr; }

#define _real_server_reg _real_psm_reg
#define _real_server_dereg _real_psm_free

#endif
