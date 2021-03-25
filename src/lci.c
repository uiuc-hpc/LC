#include "lci.h"
#include "include/lci_priv.h"
#include "include/cq.h"
#ifdef LCI_NO_PMI
#include "mpi.h"
#endif

//#define DEBUG(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
#define DEBUG(...)

lc_server** LCI_DEVICES;
LCI_endpoint_t* LCI_ENDPOINTS;
LCI_status_t LCI_INIT_STATUS = LCI_NOT_INITIALIZED;

char lcg_name[64];
int lcg_current_id = 0; int lcg_deadlock = 0;
volatile uint32_t lc_next_rdma_key = 1;
__thread int lcg_core_id = -1;

void lc_config_init(int num_proc, int rank);

LCI_error_t LCI_initialize(int* argc, char*** args)
{
  int is_init;
  is_init = PMI2_Initialized();
  if (is_init) {
	  DEBUG("Starting LCI_initialize: PMI2 is already initialized");
  } else {
	  DEBUG("Starting LCI_initialize: PMI2 is not initialized");
  }
#ifdef LCI_NO_PMI
  DEBUG("PMI is disabled.");
#endif

  MPI_Initialized(&is_init);
  if(is_init) {
	  DEBUG("Starting LCI_initialize: MPI is already initialized");
  } else {
	  DEBUG("Starting LCI_initialize: MPI is not initialized");
  }

  int num_proc, rank;
  // Initialize processes in this job.
#ifndef LCI_NO_PMI
  lc_pm_master_init(&num_proc, &rank, lcg_name);
#else
  MPI_Initialized(&is_init);
  if(!is_init) {
    MPI_Init(argc, args);
  }
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size(MPI_COMM_WORLD, &num_proc);
  // TODO: find out what lcg_name is supposed to be and set it too
#endif

  // Set some constant from environment variable.
  lc_config_init(num_proc, rank);

  for (int i = 0; i < LCI_NUM_DEVICES; i++) {
    lc_dev_init(i, &LCI_DEVICES[i]);
  }
  LCI_INIT_STATUS = LCI_INIT_COMPLETED;

  is_init = PMI2_Initialized();
  if (is_init) {
          DEBUG("Ending LCI_initialize: PMI2 is already initialized");
  } else {
          DEBUG("Ending LCI_initialize: PMI2 is not initialized");
  }
  return LCI_OK;
}

LCI_error_t LCI_finalize()
{
  for (int i = 0; i < LCI_NUM_DEVICES; i++) {
    lc_dev_finalize(LCI_DEVICES[i]);
  }
  LCI_PM_barrier();
#ifndef LCI_NO_PMI
  lc_pm_finalize();
#endif
  LCI_INIT_STATUS = LCI_FINALIZE_COMPLETED;
  return LCI_OK;
}


LCI_error_t LCI_initialized(int* flag)
{
  *flag = (LCI_INIT_STATUS >= LCI_INIT_COMPLETED);
  return LCI_OK;
}


LCI_error_t LCI_finalized(int* flag)
{
  *flag = (LCI_INIT_STATUS >= LCI_FINALIZE_COMPLETED);
  return LCI_OK;
}

LCI_error_t LCI_endpoint_create(int device, LCI_PL_t prop, LCI_endpoint_t* ep_ptr)
{
  static int num_endpoints = 0;
  struct LCI_endpoint_s* ep = 0;
  lc_server* dev = LCI_DEVICES[device];
  posix_memalign((void**) &ep, 64, sizeof(struct LCI_endpoint_s));
  ep->server = dev;
  ep->pkpool = dev->pkpool;
  ep->gid = num_endpoints++;
  LCI_ENDPOINTS[ep->gid] = ep;

  if (prop->ctype == LCI_COMM_2SIDED || prop->ctype == LCI_COMM_COLLECTIVE) {
    ep->property = EP_AR_EXP;
    ep->mt = (lc_hash*) prop->mt;
  } else {
    ep->property = EP_AR_DYN;
    ep->alloc = prop->allocator;
  }

  if (prop->rtype == LCI_COMPLETION_ONE2ONEL) {
    ep->property |= EP_CE_SYNC;
  } else if (prop->rtype == LCI_COMPLETION_HANDLER) {
    ep->property |= EP_CE_AM;
    ep->handler = prop->handler;
  } else if (prop->rtype == LCI_COMPLETION_QUEUE) {
    ep->property |= EP_CE_CQ;
    ep->cq = (lc_cq*) prop->cq;
  }

  ep->rep = dev->rep;
  *ep_ptr = ep;
  return LCI_OK;
}

LCI_error_t LCI_cq_dequeue(LCI_endpoint_t ep, LCI_request_t** req_ptr)
{
  LCI_request_t* req = lc_cq_pop(ep->cq);
  if (!req) return LCI_ERR_RETRY;
  *req_ptr = req;
  return LCI_OK;
}

LCI_error_t LCI_progress(int id, int count)
{
  lc_server_progress(LCI_DEVICES[id]);
  return LCI_OK;
}

uintptr_t LCI_get_base_addr(int id) {
  return (uintptr_t) LCI_DEVICES[id]->heap_addr;
}

LCI_error_t LCI_bbuffer_get(LCI_bbuffer_t* buffer, int device_id) {
  LC_POOL_GET_OR_RETN(LCI_DEVICES[device_id]->pkpool, p);
  *buffer = (void*) &p->data;
  return LCI_OK;
}

LCI_error_t LCI_bbuffer_free(LCI_bbuffer_t buffer, int device_id)
{
  lc_packet* packet = LC_PACKET_OF(buffer);
  lc_pool_put(LCI_DEVICES[device_id]->pkpool, packet);
  return LCI_OK;
}

void LCI_PM_barrier() {
#ifndef LCI_NO_PMI
  lc_pm_barrier();
#endif
}
