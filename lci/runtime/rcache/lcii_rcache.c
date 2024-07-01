#include "runtime/lcii.h"

#ifdef LCI_COMPILE_DREG
#include "lci_ucx_api.h"

typedef struct {
  ucs_rcache_region_t super;
  LCIS_mr_t mr;
} LCII_rcache_entry_t;

static ucs_status_t LCII_rcache_mem_reg_cb(void* context, ucs_rcache_t* rcache,
                                           void* arg,
                                           ucs_rcache_region_t* rregion,
                                           uint16_t rcache_mem_reg_flags);
static void LCII_rcache_mem_dereg_cb(void* context, ucs_rcache_t* rcache,
                                     ucs_rcache_region_t* rregion);
static void LCII_rcache_dump_region_cb(void* context, ucs_rcache_t* rcache,
                                       ucs_rcache_region_t* rregion, char* buf,
                                       size_t max);

static ucs_rcache_ops_t LCII_mem_rcache_ops = {
    .mem_reg = LCII_rcache_mem_reg_cb,
    .mem_dereg = LCII_rcache_mem_dereg_cb,
    .dump_region = LCII_rcache_dump_region_cb};

static ucs_status_t LCII_rcache_mem_reg_cb(void* context, ucs_rcache_t* rcache,
                                           void* arg,
                                           ucs_rcache_region_t* rregion,
                                           uint16_t rcache_mem_reg_flags)
{
  LCII_rcache_entry_t* region = ucs_derived_of(rregion, LCII_rcache_entry_t);
  LCI_device_t device = context;
  region->mr = LCIS_rma_reg(
      device->endpoint_progress->endpoint, (void*)region->super.super.start,
      region->super.super.end - region->super.super.start);
  return UCS_OK;
}

static void LCII_rcache_mem_dereg_cb(void* context, ucs_rcache_t* rcache,
                                     ucs_rcache_region_t* rregion)
{
  LCII_rcache_entry_t* region = ucs_derived_of(rregion, LCII_rcache_entry_t);
  LCI_device_t device = context;
  LCIS_rma_dereg(region->mr);
}

static void LCII_rcache_dump_region_cb(void* context, ucs_rcache_t* rcache,
                                       ucs_rcache_region_t* rregion, char* buf,
                                       size_t max)
{
  LCII_rcache_entry_t* region = ucs_derived_of(rregion, LCII_rcache_entry_t);

  LCIS_rkey_t rkey = LCIS_rma_rkey(region->mr);
#ifdef LCI_USE_SERVER_UCX
  snprintf(buf, max, "(%p, %lu) mr_p %p rkey [%lx,%lx]", region->mr.address,
           region->mr.length, region->mr.mr_p, rkey.val[0], rkey.val[1]);
#else
  snprintf(buf, max, "(%p, %lu) mr_p %p rkey %lx", region->mr.address,
           region->mr.length, region->mr.mr_p, LCIS_rma_rkey(region->mr));
#endif
}

LCI_error_t LCII_rcache_init(LCI_device_t device)
{
  ucs_rcache_params_t rcache_params;

  rcache_params.region_struct_size = sizeof(LCII_rcache_entry_t);
  rcache_params.max_alignment = LCI_PAGESIZE;
  rcache_params.max_unreleased = SIZE_MAX;
  rcache_params.max_regions = -1;
  rcache_params.max_size = -1;
  rcache_params.ucm_event_priority = 500; /* Default UCT pri - 1000 */
  rcache_params.ucm_events = UCM_EVENT_VM_UNMAPPED | UCM_EVENT_MEM_TYPE_FREE;
  rcache_params.context = device;
  rcache_params.ops = &LCII_mem_rcache_ops;
  rcache_params.flags = UCS_RCACHE_FLAG_PURGE_ON_FORK;
  rcache_params.alignment = UCS_RCACHE_MIN_ALIGNMENT;

  ucs_status_t ret = LCII_ucs_rcache_create(&rcache_params, "lci_rcache",
                                            LCII_ucs_stats_get_root(),
                                            (ucs_rcache_t**)&device->rcache);
  LCI_Assert(ret == UCS_OK, "Unexpected return value %d\n", ret);
  return LCI_OK;
}

void LCII_rcache_fina(LCI_device_t device)
{
  if (device->rcache != NULL) {
    LCII_ucs_rcache_destroy(device->rcache);
  }
}

void LCII_rcache_reg(LCI_device_t device, void* address, size_t length,
                     LCI_segment_t segment)
{
  ucs_status_t ret = LCII_ucs_rcache_get(
      device->rcache, address, length, PROT_READ | PROT_WRITE, NULL,
      (ucs_rcache_region_t**)&segment->region);
  LCII_rcache_entry_t* region =
      ucs_derived_of(segment->region, LCII_rcache_entry_t);
  segment->device = device;
  segment->mr = region->mr;
  LCI_Assert(ret == UCS_OK, "ucs_rcache_get failed (%d)!\n", ret);
}

LCI_error_t LCII_rcache_dereg(LCI_segment_t segment)
{
  LCII_ucs_rcache_region_put(segment->device->rcache, segment->region);
  return LCI_OK;
}
#else
LCI_error_t LCII_rcache_init(LCI_device_t device)
{
  LCI_Assert(false, "LCI_COMPILE_DREG is not enabled!\n");
  return LCI_ERR_FATAL;
}
void LCII_rcache_fina(LCI_device_t device)
{
  LCI_Assert(false, "LCI_COMPILE_DREG is not enabled!\n");
}
void LCII_rcache_reg(LCI_device_t device, void* address, size_t length,
                     LCI_segment_t segment)
{
  LCI_Assert(false, "LCI_COMPILE_DREG is not enabled!\n");
}
LCI_error_t LCII_rcache_dereg(LCI_segment_t segment)
{
  LCI_Assert(false, "LCI_COMPILE_DREG is not enabled!\n");
}
#endif