#include "runtime/lcii.h"

LCI_endpoint_t* LCI_ENDPOINTS;

// We cannot use LCI_barrier() in the implementation of LCII_barrier().
LCI_error_t LCII_endpoint_init(LCI_endpoint_t* ep_ptr, LCI_device_t device,
                               LCI_plist_t plist, bool enable_barrier)
{
  static int num_endpoints = 0;
  LCI_endpoint_t ep = LCIU_malloc(sizeof(struct LCI_endpoint_s));
  LCI_Assert(num_endpoints < LCI_MAX_ENDPOINTS, "Too many endpoints!\n");
  ep->gid = num_endpoints++;
  LCI_ENDPOINTS[ep->gid] = ep;
  *ep_ptr = ep;

  ep->device = device;
  ep->pkpool = device->heap->pool;
  ep->mt = device->mt;
  ep->ctx_archive_p = &device->ctx_archive;
  ep->bq_p = &device->bq;
  ep->bq_spinlock_p = &device->bq_spinlock;
  ep->match_type = plist->match_type;
  ep->cmd_comp_type = plist->cmd_comp_type;
  ep->msg_comp_type = plist->msg_comp_type;
  ep->default_comp = plist->default_comp;

  if (enable_barrier) LCI_barrier();

  return LCI_OK;
}

LCI_error_t LCI_endpoint_init(LCI_endpoint_t* ep_ptr, LCI_device_t device,
                              LCI_plist_t plist)
{
  return LCII_endpoint_init(ep_ptr, device, plist, true);
}

LCI_error_t LCI_endpoint_free(LCI_endpoint_t* ep_ptr)
{
  LCI_endpoint_t ep = *ep_ptr;

  LCI_ENDPOINTS[ep->gid] = NULL;
  LCIU_free(ep);

  *ep_ptr = NULL;
  return LCI_OK;
}