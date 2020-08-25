#include "lc.h"
#include "config.h"

#include "lc_priv.h"
#include "lc/pool.h"

#define LC_MED_POOL_ID(ep, size) ((size > LC_PKT_RET_MED_SIZE) ? lc_pool_get_local_id(ep->pkpool) : -1)

lc_status lc_sendm(void* src, size_t size, int rank, int tag, lc_ep ep)
{
  LC_POOL_GET_OR_RETN(ep->pkpool, p);
  lci_pk_init(ep, LC_MED_POOL_ID(ep, size), LC_PROTO_DATA, p);
  struct lci_rep* rep = &(ep->rep[rank]);
  memcpy(p->data.buffer, src, size);
  lc_server_sendm(ep->server, rep->handle, size, p,
                  MAKE_PROTO(ep->gid, LC_PROTO_DATA, tag));
  return LC_OK;
}

lc_status lc_putm(void* src, size_t size, int rank, uintptr_t addr, lc_ep ep)
{
  LC_POOL_GET_OR_RETN(ep->pkpool, p);
  lci_pk_init(ep, LC_MED_POOL_ID(ep, size), LC_PROTO_DATA, p);
  struct lci_rep* rep = &(ep->rep[rank]);
  memcpy(&p->data, src, size);
  lc_server_putm(ep->server, rep->handle, rep->base, (uint32_t) (addr - rep->base),
                 rep->rkey, size, p);
  return LC_OK;
}

lc_status lc_putms(void* src, size_t size, int rank, uintptr_t addr, int meta, lc_ep ep)
{
  LC_POOL_GET_OR_RETN(ep->pkpool, p);
  lci_pk_init(ep, LC_MED_POOL_ID(ep, size), LC_PROTO_DATA, p);
  struct lci_rep* rep = &(ep->rep[rank]);
  memcpy(&p->data, src, size);
  lc_server_putms(ep->server, rep->handle, rep->base, (uint32_t) (addr - rep->base),
                  rep->rkey, size, MAKE_PROTO(ep->gid, LC_PROTO_LONG, meta), p);
  return LC_OK;
}

lc_status lc_recvm(void* src, size_t size, int rank, int tag, lc_ep ep,
                   lc_req* req)
{
  lci_init_req(src, size, req);
  lc_key key = lc_make_key(rank, tag);
  lc_value value = (lc_value)req;
  if (!lc_hash_insert(ep->tbl, key, &value, CLIENT)) {
    lc_packet* p = (lc_packet*) value;
    memcpy(src, p->data.buffer, p->context.req->size);
    req->size = p->context.req->size;
    p->context.req = req;
    lci_ce_dispatch(ep, p, ep->cap);
  }
  return LC_OK;
}
