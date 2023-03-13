#include "lci.h"
#include "lci_priv.h"
#include "config.h"

LCI_API int LCI_IMMEDIATE_LENGTH;
LCI_API int LCI_BUFFERED_LENGTH;
LCI_API int LCI_NUM_DEVICES;
LCI_API int LCI_NUM_ENDPOINTS;
LCI_API int LCI_NUM_PROCESSES;
LCI_API int LCI_RANK;
LCI_API int LCI_REGISTERED_MEMORY_SIZE;

static inline int getenv_or(char* env, int def) {
  char* val = getenv(env);
  if (val != NULL) {
    return atoi(val);
  } else {
    return def;
  }
}

void lc_config_init(int num_proc, int rank)
{
  LCI_IMMEDIATE_LENGTH = LC_MAX_INLINE;
  LCI_BUFFERED_LENGTH = LC_PACKET_SIZE;
  LCI_NUM_DEVICES = getenv_or("LCI_NUM_DEVICES", 1);
  LCI_NUM_ENDPOINTS = getenv_or("LCI_NUM_ENDPOINTS", 8);
  LCI_NUM_PROCESSES = num_proc;
  LCI_RANK = rank;
  LCI_REGISTERED_MEMORY_SIZE = getenv_or("LCI_REGISTERED_MEMORY_SIZE", LC_DEV_MEM_SIZE);

  LCI_DEVICES = calloc(sizeof(lc_server*), LCI_NUM_DEVICES);
  LCI_ENDPOINTS = calloc(sizeof(LCI_endpoint_t), LCI_NUM_ENDPOINTS);
}
