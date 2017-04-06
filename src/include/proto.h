#ifndef MV_PROTO_H
#define MV_PROTO_H

#define RDZ_MATCH_TAG ((uint32_t) 1 << 24)

#define INIT_CTX(ctx)         \
  {                           \
    ctx->buffer = (void*)src; \
    ctx->size = size;         \
    ctx->rank = rank;         \
    ctx->tag = tag;           \
    ctx->sync = 0; \
    ctx->type = REQ_PENDING;  \
    ctx->lock = 0; \
  }

#define MAKE_PROTO(proto, tag) (((uint32_t) proto) | ((uint32_t)tag << 8))

typedef struct {
  lc_am_func_t func_am;
  lc_am_func_t func_ps;
} lc_proto_spec_t;

const lc_proto_spec_t lc_proto[10] __attribute__((aligned(64)));

MV_INLINE
int lci_send(lch* mv, const void* src, int size, int rank, int tag,
            uint32_t proto, lc_packet* p)
{
  p->context.poolid = (size > 128)?lc_pool_get_local(mv->pkpool):0;
  return lc_server_send(mv->server, rank, (void*) src, size,
                        p, MAKE_PROTO(proto, tag));
}

MV_INLINE void lci_rdma_match(lch* mv, lc_packet* p, lc_ctx* ctx)
{
  p->context.req = (uintptr_t)ctx;
  lc_server_rma_signal(mv->server, p->context.from, ctx->buffer,
                       p->data.rtr.tgt_addr, p->data.rtr.rkey,
                       ctx->size, p->data.rtr.comm_id, p, MV_PROTO_LONG_MATCH);
}

MV_INLINE void lci_rdz_prepare(lch* mv, void* src, int size, lc_ctx* ctx, lc_packet* p)
{
  p->context.req = (uintptr_t)ctx;
  uintptr_t rma_mem = lc_server_rma_reg(mv->server, src, size);
  p->context.rma_mem = rma_mem;
  p->data.rtr.comm_id =
      (uint32_t)((uintptr_t)p - (uintptr_t)lc_heap_ptr(mv));
  p->data.rtr.tgt_addr = (uintptr_t)src;
  p->data.rtr.rkey = lc_server_rma_key(rma_mem);
}

MV_INLINE
void lc_serve_recv(lch* mv, lc_packet* p_ctx, uint32_t proto)
{
  lc_proto[proto].func_am(mv, p_ctx);
}

MV_INLINE
void lc_serve_send(lch* mv, lc_packet* p_ctx, uint32_t proto)
{
  const lc_am_func_t f = lc_proto[proto].func_ps;
  if (likely(f)) {
    f(mv, p_ctx);
  }
}

MV_INLINE
void lc_serve_imm(lch* mv, uint32_t imm)
{
  // FIXME(danghvu): This comm_id is here due to the imm
  // only takes uint32_t, if this takes uint64_t we can
  // store a pointer to this request context.
  if (imm & RMA_SIGNAL_QUEUE) {
    imm ^= RMA_SIGNAL_QUEUE;
    lc_packet* p = (lc_packet*)((uintptr_t)lc_heap_ptr(mv) + imm);
    lc_ctx* req = (lc_ctx*)p->context.req;
    lc_server_rma_dereg(p->context.rma_mem);
    lc_pool_put(mv->pkpool, p);
    req->type = REQ_DONE;
  } else if (imm & RMA_SIGNAL_SIMPLE) {
    imm ^= RMA_SIGNAL_SIMPLE;
    struct lc_rma_ctx* ctx =
        (struct lc_rma_ctx*)((uintptr_t)lc_heap_ptr(mv) + imm);
    if (ctx->req) ((lc_ctx*)ctx->req)->type = REQ_DONE;
  } else {
    lc_packet* p = (lc_packet*)((uintptr_t)lc_heap_ptr(mv) + imm);
    lc_ctx* req = (lc_ctx*)p->context.req;
    lc_pool_put(mv->pkpool, p);
    lc_key key = lc_make_key(req->rank, req->tag);
    lc_value value = 0;
    if (!lc_hash_insert(mv->tbl, key, &value)) {
      req->type = REQ_DONE;
      if (req->sync) thread_signal(req->sync);
    }
  }
}
#endif
