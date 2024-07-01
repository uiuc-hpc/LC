#include "lci.h"
#include "runtime/lcii.h"

LCI_error_t LCI_memory_register(LCI_device_t device, void* address,
                                size_t length, LCI_segment_t* segment)
{
  LCII_PCOUNTER_START(mem_reg_timer);
  LCI_segment_t mr = (LCI_segment_t)LCIU_malloc(sizeof(struct LCII_mr_t));
  if (LCI_USE_DREG)
    LCII_rcache_reg(device, address, length, mr);
  else {
    mr->mr = LCIS_rma_reg(device->endpoint_progress->endpoint, address, length);
  }
  *segment = mr;
  LCII_PCOUNTER_END(mem_reg_timer);
  return LCI_OK;
}

LCI_error_t LCI_memory_deregister(LCI_segment_t* segment)
{
  LCI_DBG_Assert(*segment != NULL, "*segment is NULL\n");
  LCII_PCOUNTER_START(mem_dereg_timer);
  if (LCI_USE_DREG) {
    LCII_rcache_dereg(*segment);
  } else {
    LCIS_rma_dereg((*segment)->mr);
  }
  LCIU_free(*segment);
  *segment = NULL;
  LCII_PCOUNTER_END(mem_dereg_timer);
  return LCI_OK;
}

LCI_error_t LCI_mbuffer_alloc(LCI_device_t device, LCI_mbuffer_t* mbuffer)
{
  LCII_packet_t* packet = LCII_alloc_packet_nb(device->heap->pool);
  if (packet == NULL)
    // no packet is available
    return LCI_ERR_RETRY;

  mbuffer->address = packet->data.address;
  mbuffer->length = LCI_MEDIUM_SIZE;
  LCI_DBG_Assert(LCII_is_packet(device->heap, mbuffer->address), "");
  return LCI_OK;
}

LCI_error_t LCI_mbuffer_free(LCI_mbuffer_t mbuffer)
{
  LCII_packet_t* packet = LCII_mbuffer2packet(mbuffer);
  LCII_free_packet(packet);
  return LCI_OK;
}

LCI_error_t LCI_lbuffer_alloc(LCI_device_t device, size_t size,
                              LCI_lbuffer_t* lbuffer)
{
  lbuffer->length = size;
  lbuffer->address = LCIU_malloc(size);
  if (LCI_TOUCH_LBUFFER) {
    char* p = (char*)lbuffer->address;
    if ((uint64_t)p % LCI_PAGESIZE != 0) *(char*)p = 'a';
    p = (char*)((uintptr_t)(p + LCI_PAGESIZE - 1) / LCI_PAGESIZE *
                LCI_PAGESIZE);
    for (; p < (char*)lbuffer->address + size; p += LCI_PAGESIZE) {
      *(char*)p = 'a';
    }
  }
  LCI_memory_register(device, lbuffer->address, lbuffer->length,
                      &lbuffer->segment);
  return LCI_OK;
}

LCI_error_t LCI_lbuffer_memalign(LCI_device_t device, size_t size,
                                 size_t alignment, LCI_lbuffer_t* lbuffer)
{
  lbuffer->length = size;
  lbuffer->address = LCIU_memalign(alignment, size);
  if (LCI_TOUCH_LBUFFER) {
    char* p = (char*)lbuffer->address;
    if ((uint64_t)p % LCI_PAGESIZE != 0) *(char*)p = 'a';
    p = (char*)((uintptr_t)(p + LCI_PAGESIZE - 1) / LCI_PAGESIZE *
                LCI_PAGESIZE);
    for (; p < (char*)lbuffer->address + size; p += LCI_PAGESIZE) {
      *(char*)p = 'a';
    }
  }
  LCI_memory_register(device, lbuffer->address, lbuffer->length,
                      &lbuffer->segment);
  return LCI_OK;
}

LCI_error_t LCI_lbuffer_free(LCI_lbuffer_t lbuffer)
{
  LCI_memory_deregister(&lbuffer.segment);
  LCIU_free(lbuffer.address);
  return LCI_OK;
}