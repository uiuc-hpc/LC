#ifndef LC_PROTO_H
#define LC_PROTO_H

#include "config.h"

#include "debug.h"
#include "lc/hashtable.h"
#include "lc/macro.h"

extern struct lci_ep* lcg_ep[];

/* 2-bit is enough for those thing. */
typedef enum lc_proto {
 LC_PROTO_DATA  = 0,
 LC_PROTO_RTR   = 1,
 LC_PROTO_RTS   = 2,
 LC_PROTO_LONG  = 3,
} lc_proto;

#include "server/server.h"

#define MAKE_PROTO(rgid, proto, meta)  (proto | (rgid << 2) | (meta << 16))

#define PROTO_GET_PROTO(proto) (proto         & 0b011)
#define PROTO_GET_RGID(proto)  ((proto >> 2)  & 0b011111111111111)
#define PROTO_GET_META(proto)  ((proto >> 16) & 0xffff)

/* CRC-32C (iSCSI) polynomial in reversed bit order. */
#define POLY 0x82f63b78
inline uint32_t crc32c(char *buf, size_t len)
{
  uint32_t crc = 0;
  int k;

  crc = ~crc;
  while (len--) {
    crc ^= *buf++;
    for (k = 0; k < 8; k++)
      crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
  }
  return ~crc;
}

static inline void lci_init_req(void* buf, size_t size, lc_req* req)
{
  req->buffer = buf;
  req->size = size;
  lc_reset((void*) &req->sync);
}

static inline void lci_prepare_rtr(struct lci_ep* ep, void* src, size_t size, lc_packet* p)
{
  uintptr_t rma_mem = lc_server_rma_reg(ep->server, src, size);
  p->context.rma_mem = rma_mem;
  p->context.ref = 2;
  p->data.rtr.comm_id = ((uintptr_t)p - (uintptr_t) lc_server_heap_ptr(ep->server));
  p->data.rtr.tgt_addr = (uintptr_t)src;
  p->data.rtr.rkey = lc_server_rma_key(rma_mem);

  dprintf("%d] post recv rdma %p %d via %d\n", lcg_rank, src, size, p->data.rtr.rkey);
}

static inline void lci_prepare_rts(void* src, size_t size, lc_send_cb cb, void* ce, lc_packet* p)
{
  p->data.rts.src_addr = (uintptr_t) src;
  p->data.rts.size = size;
  p->data.rts.cb = cb;
  p->data.rts.ce = (uintptr_t) ce;
  lc_mem_fence();
}

static inline void lci_pk_free(lc_ep ep, lc_packet* p)
{
  lc_pool_put(ep->pkpool, p);
}

static inline void lci_pk_free_data(lc_ep ep, lc_packet* p)
{
  if (p->context.poolid != -1)
    lc_pool_put_to(ep->pkpool, p, p->context.poolid);
  else
    lc_pool_put(ep->pkpool, p);
}

static inline void lci_ce_glob(lc_ep ep)
{
  __sync_fetch_and_add(&ep->completed, 1);
}

static inline void lci_ce_am(lc_ep ep, lc_packet* p)
{
  lc_req* req = p->context.req;
  ep->handler(req);
  lci_pk_free_data(ep, p);
}

static inline void lci_ce_signal(lc_ep ep, lc_packet* p)
{
  lc_req* req = p->context.req;
  lc_signal((void*) &(req->sync));
  lci_pk_free_data(ep, p);
}

static inline void lci_ce_queue(lc_ep ep, lc_packet* p)
{
  lc_req* req = p->context.req;
  req->parent = p;
  cq_push(&ep->cq, req);
}

static inline void lci_handle_rtr(struct lci_ep* ep, lc_packet* p)
{
  dprintf("Recv RTR %p\n", p);
  lci_pk_init(ep, -1, LC_PROTO_LONG, p);
  // dprintf("%d] rma %p --> %p %.4x via %d\n", lcg_rank, p->data.rts.src_addr, p->data.rtr.tgt_addr, crc32c((char*) p->data.rts.src_addr, p->data.rts.size), p->data.rtr.rkey);

  lc_server_rma_rtr(ep->server, p->context.req->rhandle,
      (void*) p->data.rts.src_addr,
      p->data.rtr.tgt_addr, p->data.rtr.rkey, p->data.rts.size,
      p->data.rtr.comm_id, p);
}

static inline void lci_handle_rts(struct lci_ep* ep, lc_packet* p)
{
  dprintf("Recv RTS: %p\n", p);
  lci_pk_init(ep, -1, LC_PROTO_RTR, p);
  lc_proto proto = MAKE_PROTO(ep->gid, LC_PROTO_RTR, 0);
  lci_prepare_rtr(ep, p->context.req->buffer, p->data.rts.size, p);
  lc_server_sendm(ep->server, p->context.req->rhandle,
      sizeof(struct packet_rtr), p, proto);
}

static inline void lci_ce_dispatch(lc_ep ep, lc_packet* p, const long cap);

static inline void lci_serve_recv_imm(struct lci_ep* ep, lc_packet* p, lc_proto proto, const long cap)
{
  p->context.req->meta = PROTO_GET_META(proto);
  proto = PROTO_GET_PROTO(proto);
  p->context.req->buffer = &p->data;
  lci_ce_dispatch(ep, p, cap);
}

static inline void lci_serve_recv_dyn(struct lci_ep* ep, lc_packet* p, lc_proto proto, const long cap)
{
  p->context.req->meta = PROTO_GET_META(proto);
  proto = PROTO_GET_PROTO(proto);

  if (proto == LC_PROTO_DATA) {
    void* buf = ep->alloc(p->context.req->size, &(p->context.req->ctx));
    memcpy(buf, &p->data, p->context.req->size);
    p->context.req->buffer = buf;
    lci_ce_dispatch(ep, p, cap);
  } else if (proto == LC_PROTO_RTS) {
    void* buf = ep->alloc(p->data.rts.size, &(p->context.req->ctx));
    lci_init_req(buf, p->data.rts.size, p->context.req);
    lci_handle_rts(ep, p);
  } else if (proto == LC_PROTO_RTR) {
    lci_handle_rtr(ep, p);
  };
}

static inline void lci_serve_recv_expl(struct lci_ep* ep, lc_packet* p, lc_proto proto, const long cap)
{
  p->context.req->meta = PROTO_GET_META(proto);
  p->context.proto = proto = PROTO_GET_PROTO(proto);

  if (proto == LC_PROTO_DATA) {
    const lc_key key = lc_make_key(p->context.req->rank, p->context.req->meta);
    lc_value value = (lc_value)p;
    if (!lc_hash_insert(ep->tbl, key, &value, SERVER)) {
      lc_req* req = (lc_req*) value;
      req->size = p->context.req->size;
      memcpy(req->buffer, p->data.buffer, p->context.req->size);
      p->context.req = req;
      lci_ce_dispatch(ep, p, cap);
    }
  } else if (proto == LC_PROTO_RTS) {
    const lc_key key = lc_make_key(p->context.req->rank, p->context.req->meta);
    lc_value value = (lc_value)p;
    if (!lc_hash_insert(ep->tbl, key, &value, SERVER)) {
      p->context.req = (lc_req*) value;
      lci_handle_rts(ep, p);
    }
  } else if (proto == LC_PROTO_RTR) {
    lci_handle_rtr(ep, p);
  };
}

static inline void lci_serve_recv_dispatch(lc_ep ep, lc_packet* p, lc_proto proto, const long cap)
{
#ifdef LC_SERVER_HAS_EXP
  if (cap & EP_AR_EXP) {
    return lci_serve_recv_expl(ep, p, proto, cap);
  } else
#endif
#ifdef LC_SERVER_HAS_DYN
  if (cap & EP_AR_DYN) {
    return lci_serve_recv_dyn(ep, p, proto, cap);
  } else
#endif
#ifdef LC_SERVER_HAS_IMM
  if (cap & EP_AR_IMM) {
    return lci_serve_recv_imm(ep, p, proto, cap);
  } else
#endif
  // placeholder for anything else.
  {
  }
}

static inline void lci_ce_dispatch(lc_ep ep, lc_packet* p, const long cap)
{
#ifdef LC_SERVER_HAS_SYNC
  if (cap & EP_CE_SYNC) {
    lci_ce_signal(ep, p);
  } else
#endif
#ifdef LC_SERVER_HAS_CQ
  if (cap & EP_CE_CQ) {
    lci_ce_queue(ep, p);
  } else
#endif
#ifdef LC_SERVER_HAS_AM
  if (cap & EP_CE_AM) {
    lci_ce_am(ep, p);
  } else
#endif
  {
    // If nothing is worth, return this to packet pool.
    lci_pk_free_data(ep, p);
  }

#ifdef LC_SERVER_HAS_GLOB
  if (cap & EP_CE_GLOB) {
    lci_ce_glob(ep);
  }
#endif
}

static inline void lci_serve_recv(lc_packet* p, lc_proto proto)
{
  // NOTE: this should be RGID because it is received from remote.
  struct lci_ep* ep = lcg_ep[PROTO_GET_RGID(proto)];
  return lci_serve_recv_dispatch(ep, p, proto, ep->cap);
}

static inline void lci_serve_recv_rdma(lc_packet* p, lc_proto proto)
{
  struct lci_ep* ep = lcg_ep[PROTO_GET_RGID(proto)];
  p->context.req->meta = PROTO_GET_META(proto);
  lci_ce_dispatch(ep, p, ep->cap);
}

static inline void lci_serve_send(lc_packet* p)
{
  struct lci_ep* ep = p->context.ep;
  lc_proto proto = p->context.proto;

  if (proto == LC_PROTO_RTR) {
    if (--p->context.ref == 0)
      lci_ce_dispatch(ep, p, ep->cap);
    // Have to keep the ref counting here, otherwise there is a nasty race
    // when the RMA is done and this RTR is not.
    // Note that this messed up the order of completion.
    // If one expects MPI order, should need to split completion here.
  } else if (proto == LC_PROTO_LONG) {
    dprintf("SENT LONG: %p\n", p);
    p->data.rts.cb((void*) p->data.rts.ce);
    lci_pk_free(ep, p);
  } else if (proto == LC_PROTO_RTS) {
    dprintf("SENT RTS: %p\n", p);
    lci_pk_free(ep, p);
  } else {
    dprintf("SENT UNKNOWN: %p\n", p);
    lci_pk_free_data(ep, p);
  }
}

static inline void lci_serve_imm(lc_packet* p)
{
  struct lci_ep* ep = p->context.ep;
  dprintf("%d] got %p %.4x\n", lcg_rank, p->context.req->buffer, crc32c(p->context.req->buffer, p->context.req->size));
  dprintf("Recv RDMA: %p %d\n", p, lc_server_rma_key(p->context.rma_mem));
  lc_server_rma_dereg(p->context.rma_mem);
  if (--p->context.ref == 0)
    lci_ce_dispatch(ep, p, ep->cap);
}

#endif
