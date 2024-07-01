#include "runtime/lcii.h"

LCI_error_t LCII_fill_rq(LCII_endpoint_t* endpoint, bool block);

void LCII_device_endpoint_init(LCI_device_t device, bool single_threaded,
                               LCII_endpoint_t** endpoint_pp)
{
  // This is not LCI_endpoint_t which is just a wrapper of parameters,
  // but LCII_endpoint_t which maps to an underlying network context.
  LCII_endpoint_t* endpoint_p = LCIU_malloc(sizeof(LCII_endpoint_t));
  *endpoint_pp = endpoint_p;
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  atomic_init(&endpoint_p->recv_posted, 0);
#else
  endpoint_p->recv_posted = 0;
#endif
  endpoint_p->device = device;
  LCIS_endpoint_init(g_server, &endpoint_p->endpoint, single_threaded);
}

void LCII_endpoint_fina(LCII_endpoint_t** endpoint_pp)
{
  LCII_endpoint_t* endpoint_p = *endpoint_pp;
  LCIS_endpoint_fina(endpoint_p->endpoint);
  LCIU_free(endpoint_p);
  *endpoint_pp = NULL;
}

int LCII_endpoint_get_recv_posted(LCII_endpoint_t* endpoint_p)
{
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  return atomic_load(&endpoint_p->recv_posted);
#else
  return endpoint_p->recv_posted;
#endif
}

LCI_error_t LCI_device_init(LCI_device_t* device_ptr)
{
  LCI_device_t device = LCIU_malloc(sizeof(struct LCI_device_s));
  *device_ptr = device;

  bool single_threaded_prg = true;
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  single_threaded_prg = false;
#endif
  LCII_device_endpoint_init(device, false, &device->endpoint_worker);
  if (LCI_ENABLE_PRG_NET_ENDPOINT) {
    LCII_device_endpoint_init(device, single_threaded_prg,
                              &device->endpoint_progress);
  } else {
    device->endpoint_progress = device->endpoint_worker;
  }
  if (LCI_USE_DREG) {
    LCII_rcache_init(device);
  }

  LCII_matchtable_create(&device->mt);
  LCM_archive_init(&(device->ctx_archive), 16);
  LCII_bq_init(&device->bq);
  LCIU_spinlock_init(&device->bq_spinlock);

  device->heap = &g_heap;
  LCI_memory_register(device, device->heap->address, device->heap->length,
                      &device->heap_segment);

  if (LCI_ENABLE_PRG_NET_ENDPOINT)
    LCII_fill_rq(device->endpoint_progress, true);
  LCII_fill_rq(device->endpoint_worker, true);
  LCI_barrier();
  LCI_Log(LCI_LOG_INFO, "device", "device %p initialized\n", device);
  return LCI_OK;
}

LCI_error_t LCI_device_initx(LCI_device_t* device_ptr)
{
  return LCI_device_init(device_ptr);
}

LCI_error_t LCI_device_free(LCI_device_t* device_ptr)
{
  LCI_device_t device = *device_ptr;
  LCI_Log(LCI_LOG_INFO, "device", "free device %p\n", device);
  LCI_barrier();
  device->heap->total_recv_posted +=
      LCII_endpoint_get_recv_posted(device->endpoint_worker);
  if (LCI_ENABLE_PRG_NET_ENDPOINT) {
    device->heap->total_recv_posted +=
        LCII_endpoint_get_recv_posted(device->endpoint_progress);
  }
  LCI_memory_deregister(&device->heap_segment);
  LCII_matchtable_free(&device->mt);
  LCM_archive_fini(&(device->ctx_archive));
  LCII_bq_fini(&device->bq);
  LCIU_spinlock_fina(&device->bq_spinlock);
  if (LCI_USE_DREG) {
    LCII_rcache_fina(device);
  }
  if (LCI_ENABLE_PRG_NET_ENDPOINT) {
    LCII_endpoint_fina(&device->endpoint_progress);
  }
  LCII_endpoint_fina(&device->endpoint_worker);
  LCIU_free(device);
  *device_ptr = NULL;
  return LCI_OK;
}