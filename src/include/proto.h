#ifndef LC_PROTO_H
#define LC_PROTO_H

#include "config.h"

#include "lci.h"
#include "debug.h"
#include "cq.h"
#include "hashtable.h"
#include "macro.h"

/* 2-bit is enough for those thing. */
typedef enum lc_proto {
  LC_PROTO_DATA = 0,
  LC_PROTO_RTR = 1,
  LC_PROTO_RTS = 2,
  LC_PROTO_LONG = 3,
} lc_proto;

#include "server/server.h"

#define MAKE_PROTO(rgid, proto, tag) (proto | (rgid << 2) | (tag << 16))

#define PROTO_GET_PROTO(proto) (proto & 0b011)
#define PROTO_GET_RGID(proto) ((proto >> 2) & 0b011111111111111)
#define PROTO_GET_META(proto) ((proto >> 16) & 0xffff)

/* CRC-32C (iSCSI) polynomial in reversed bit order. */
#define POLY 0x82f63b78
inline uint32_t crc32c(char* buf, size_t len)
{
  uint32_t crc = 0;
  int k;

  crc = ~crc;
  while (len--) {
    crc ^= *buf++;
    for (k = 0; k < 8; k++) crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
  }
  return ~crc;
}

static inline void lc_init_req(void* buf, size_t size, LCI_request_t* req)
{
  req->buffer.dbuffer = buf;
  req->type = DIRECT;
  req->length = size;
}

static inline void lc_prepare_rtr(LCI_endpoint_t ep, void* src, size_t size,
                                  lc_packet* p)
{
  uintptr_t rma_mem = lc_server_rma_reg(ep->server, src, size);
  p->context.rma_mem = rma_mem;
  p->context.ref = 2;
  p->data.rtr.comm_id =
      ((uintptr_t)p - (uintptr_t)lc_server_heap_ptr(ep->server));
  p->data.rtr.tgt_addr = (uintptr_t)src;
  p->data.rtr.rkey = lc_server_rma_key(rma_mem);

  dprintf("%d] post recv rdma %p %d via %d\n", lcg_rank, src, size,
          p->data.rtr.rkey);
}

static inline void lc_prepare_rts(void* src, size_t size, void* usr_context,
                                  lc_packet* p)
{
  p->data.rts.src_addr = (uintptr_t)src;
  p->data.rts.size = size;
  p->data.rts.ce = (uintptr_t)usr_context;
  lc_mem_fence();
}

static inline void lc_ce_glob(LCI_endpoint_t ep)
{
  __sync_fetch_and_add(&ep->completed, 1);
}

static inline void lc_ce_am(LCI_endpoint_t ep, lc_packet* p, void* sync)
{
  ep->handler(sync, 0);
  lc_pk_free_data(ep, p);
}

static inline void lc_ce_signal(LCI_endpoint_t ep, lc_packet* p, void* sync)
{
  LCI_one2one_set_full(sync);
  lc_pk_free_data(ep, p);
}

static inline void lc_ce_queue(LCI_endpoint_t ep, lc_packet* p, void* sync)
{
  LCI_request_t* req = LCI_SYNCL_PTR_TO_REQ_PTR(sync);
  req->buffer.bbuffer = &p->data;
  req->type = BUFFERED;
  lc_cq_push(ep->cq, req);
}

static inline void lc_handle_rtr(LCI_endpoint_t ep, lc_packet* p)
{
  dprintf("Recv RTR %p\n", p);
  lc_pk_init(ep, -1, LC_PROTO_LONG, p);

  lc_server_rma_rtr(ep->server, p->context.sync->request.__reserved__,
                    (void*)p->data.rts.src_addr, p->data.rtr.tgt_addr,
                    p->data.rtr.rkey, p->data.rts.size, p->data.rtr.comm_id, p);
}

static inline void lc_handle_rts(LCI_endpoint_t ep, lc_packet* p)
{
  dprintf("Recv RTS: %p\n", p);
  lc_pk_init(ep, -1, LC_PROTO_RTR, p);
  lc_proto proto = MAKE_PROTO(ep->gid, LC_PROTO_RTR, 0);
  lc_prepare_rtr(ep, p->context.sync->request.buffer.dbuffer, p->data.rts.size, p);
  dprintf("Send RTR: %p\n", p, p->context.sync->request.__reserved__, proto);
  lc_server_sendm(ep->server, p->context.sync->request.__reserved__,
                  sizeof(struct packet_rtr), p, proto);
}

static inline void lc_ce_dispatch(LCI_endpoint_t ep, lc_packet* p, void* sync,
                                  const long cap);

static inline void lc_serve_recv_imm(LCI_endpoint_t ep, lc_packet* p,
                                     uint16_t tag, const long cap)
{
  p->context.sync->request.tag = tag;
  p->context.sync->request.buffer.bbuffer = &p->data;
  lc_ce_dispatch(ep, p, p->context.sync, cap);
}

static inline void lc_serve_recv_dyn(LCI_endpoint_t ep, lc_packet* p,
                                     uint32_t proto, uint16_t tag, const long cap)
{
  p->context.sync->request.tag = tag;

  if (proto == LC_PROTO_DATA) {
    void* buf = ep->alloc(p->context.sync->request.length, 0);
    memcpy(buf, &p->data, p->context.sync->request.length);
    p->context.sync->request.buffer.dbuffer = buf;
    lc_ce_dispatch(ep, p, p->context.sync, cap);
  } else if (proto == LC_PROTO_RTS) {
    void* buf = ep->alloc(p->data.rts.size, 0);
    lc_init_req(buf, p->data.rts.size, &p->context.sync->request);
    lc_handle_rts(ep, p);
  } else if (proto == LC_PROTO_RTR) {
    lc_handle_rtr(ep, p);
  };
}

static inline void lc_serve_recv_match(LCI_endpoint_t ep, lc_packet* p,
                                      uint32_t proto, uint16_t tag, const long cap)
{
  p->context.sync->request.tag = tag;
  p->context.proto = proto;

  if (proto == LC_PROTO_DATA) {
    const lc_key key = lc_make_key(p->context.sync->request.rank,
                                   p->context.sync->request.tag);
    lc_value value = (lc_value)p;
    if (!lc_hash_insert(ep->mt, key, &value, SERVER)) {
      LCI_syncl_t* sync = (LCI_syncl_t*)value;
      sync->request.length = p->context.sync->request.length;
      memcpy(sync->request.buffer.dbuffer, p->data.buffer,
             p->context.sync->request.length);
      lc_ce_dispatch(ep, p, sync, cap);
    }
  } else if (proto == LC_PROTO_RTS) {
    const lc_key key = lc_make_key(p->context.sync->request.rank,
                                   p->context.sync->request.tag);
    lc_value value = (lc_value)p;
    if (!lc_hash_insert(ep->mt, key, &value, SERVER)) {
      p->context.sync = (LCI_syncl_t*)value;
      lc_handle_rts(ep, p);
    }
  } else if (proto == LC_PROTO_RTR) {
    lc_handle_rtr(ep, p);
  };
}

static inline void lc_serve_recv_dispatch(LCI_endpoint_t ep, lc_packet* p,
                                          uint32_t proto, uint16_t meta, const long cap)
{
  if (proto == LC_PROTO_LONG) {
    lc_serve_recv_imm(ep, p, meta, cap);
    return;
  }

#ifdef LCI_SERVER_HAS_EXP
  if (cap & EP_AR_EXP) {
    return lc_serve_recv_match(ep, p, proto, meta, cap);
  } else
#endif
#ifdef LCI_SERVER_HAS_DYN
  if (cap & EP_AR_DYN) {
    return lc_serve_recv_dyn(ep, p, proto, meta, cap);
  } else
#endif
#ifdef LCI_SERVER_HAS_IMM
  if (cap & EP_AR_IMM) {
    return lc_serve_recv_imm(ep, p, meta, cap);
  } else
#endif
  // placeholder for anything else.
  {
  }
}

static inline void lc_ce_dispatch(LCI_endpoint_t ep, lc_packet* p, void* sync,
                                  const long cap)
{
#ifdef LCI_SERVER_HAS_SYNC
  if (cap & EP_CE_SYNC) {
    lc_ce_signal(ep, p, sync);
  } else
#endif
#ifdef LCI_SERVER_HAS_CQ
  if (cap & EP_CE_CQ) {
    lc_ce_queue(ep, p, sync);
  } else
#endif
#ifdef LCI_SERVER_HAS_AM
      if (cap & EP_CE_AM) {
    lc_ce_am(ep, p, sync);
  } else
#endif
  {
    // If nothing is worth, return this to packet pool.
    lc_pk_free_data(ep, p);
  }

#ifdef LCI_SERVER_HAS_GLOB
  if (cap & EP_CE_GLOB) {
    lc_ce_glob(ep);
  }
#endif
}

static inline void lc_serve_recv(lc_packet* p, lc_proto raw_proto)
{
  // NOTE: this should be RGID because it is received from remote.
  LCI_endpoint_t ep = LCI_ENDPOINTS[PROTO_GET_RGID(raw_proto)];
  uint16_t meta = PROTO_GET_META(raw_proto);
  uint32_t proto = PROTO_GET_PROTO(raw_proto);
  return lc_serve_recv_dispatch(ep, p, proto, meta, ep->property);
}

static inline void lc_serve_recv_rdma(lc_packet* p, lc_proto proto)
{
  LCI_endpoint_t ep = LCI_ENDPOINTS[PROTO_GET_RGID(proto)];
  p->context.sync->request.tag = PROTO_GET_META(proto);
  lc_ce_dispatch(ep, p, p->context.sync, ep->property);
}

static inline void lc_serve_send(lc_packet* p)
{
  LCI_endpoint_t ep = p->context.ep;
  lc_proto proto = p->context.proto;

  if (proto == LC_PROTO_RTR) {
    if (--p->context.ref == 0)
      lc_ce_dispatch(ep, p, p->context.sync, ep->property);
    // Have to keep the ref counting here, otherwise there is a nasty race
    // when the RMA is done and this RTR is not.
    // Note that this messed up the order of completion.
    // If one expects MPI order, should need to split completion here.
  } else if (proto == LC_PROTO_LONG) {
    dprintf("SENT LONG: %p\n", p);
    LCI_one2one_set_full((LCI_sync_t*)p->data.rts.ce);  // FIXME
    lc_pk_free(ep, p);
  } else if (proto == LC_PROTO_RTS) {
    dprintf("SENT RTS: %p\n", p);
    lc_pk_free(ep, p);
  } else {
    dprintf("SENT UNKNOWN: %p\n", p);
    if (p->context.ref != USER_MANAGED) {
      lc_pk_free_data(ep, p);
    } else {
      LCI_one2one_set_full(p->context.sync);  // FIXME
    }
  }
}

static inline void lc_serve_imm(lc_packet* p)
{
  LCI_endpoint_t ep = p->context.ep;
  dprintf("Recv RDMA: %p %d\n", p, lc_server_rma_key(p->context.rma_mem));
  lc_server_rma_dereg(p->context.rma_mem);
  if (--p->context.ref == 0)
    lc_ce_dispatch(ep, p, p->context.sync, ep->property);
}

#endif
