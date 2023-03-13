#include "config.h"
#include "lci.h"
#include "lci_priv.h"

size_t lc_max_short(int dev __UNUSED__)
{
  return LC_MAX_INLINE;
}

size_t lc_max_medium(int dev __UNUSED__)
{
  return LC_PACKET_SIZE;
}

#ifdef USE_DREG
uintptr_t get_dma_mem(void* server, void* buf, size_t s)
{
  return _real_server_reg(server, buf, s);
}

int free_dma_mem(uintptr_t mem)
{
  _real_server_dereg(mem);
  return 1;
}
#endif
