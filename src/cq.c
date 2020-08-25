#include "lc.h"

#include "lc_priv.h"
#include "lc/pool.h"

lc_status lc_cq_pop(lc_ep ep, lc_req** req_ptr)
{
  lc_req* req = cq_pop(&ep->cq);
  if (!req) return LC_ERR_RETRY;
  *req_ptr = req;
  return LC_OK;
}

lc_status lc_cq_reqfree(lc_ep ep, lc_req* req)
{
  lc_packet* packet = (lc_packet*) req->parent;
  lci_pk_free_data(ep, packet);
  return LC_OK;
}
