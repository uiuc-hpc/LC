#include "runtime/lcii.h"
#ifdef LCI_COMPILE_DREG
#include "lci_ucx_api.h"
#endif

static int opened = 0;
int LCIU_nthreads = 0;
__thread int LCIU_thread_id = -1;
__thread unsigned int LCIU_rand_seed = 0;
LCIS_server_t g_server;
LCII_packet_heap_t g_heap;

void initialize_packet_heap(LCII_packet_heap_t* heap)
{
  heap->length = LCI_CACHE_LINE + (size_t)LCI_SERVER_NUM_PKTS * LCI_PACKET_SIZE;
  heap->address = LCIU_memalign(LCI_PAGESIZE, heap->length);
  LCI_Assert(LCI_CACHE_LINE >= sizeof(struct LCII_packet_context),
             "packet context is too large!\n");
  heap->base_packet_p =
      heap->address + LCI_CACHE_LINE - sizeof(struct LCII_packet_context);
  LCI_Assert(LCI_PACKET_SIZE % LCI_CACHE_LINE == 0,
             "The size of packets should be a multiple of cache line size\n");
  LCII_pool_create(&heap->pool);
  for (size_t i = 0; i < LCI_SERVER_NUM_PKTS; i++) {
    LCII_packet_t* packet =
        (LCII_packet_t*)(heap->base_packet_p + i * LCI_PACKET_SIZE);
    LCI_Assert(((uint64_t) & (packet->data)) % LCI_CACHE_LINE == 0,
               "packet.data is not well-aligned\n");
    LCI_Assert(LCII_is_packet(heap, packet->data.address),
               "Not a packet. The computation is wrong!\n");
    packet->context.pkpool = heap->pool;
    packet->context.poolid = 0;
#ifdef LCI_DEBUG
    packet->context.isInPool = true;
#endif
    LCII_pool_put(heap->pool, packet);
  }
  LCI_Assert(LCI_SERVER_NUM_PKTS > 2 * LCI_SERVER_MAX_RECVS,
             "The packet number is too small!\n");
  heap->total_recv_posted = 0;
}

void finalize_packet_heap(LCII_packet_heap_t* heap)
{
  int total_num = LCII_pool_count(heap->pool) + heap->total_recv_posted;
  if (total_num != LCI_SERVER_NUM_PKTS)
    LCI_Warn("Potentially losing packets %d != %d\n", total_num,
             LCI_SERVER_NUM_PKTS);
  LCII_pool_destroy(heap->pool);
  LCIU_free(heap->address);
}

LCI_error_t LCI_initialize()
{
  if (getenv("LCI_INIT_ATTACH_DEBUGGER")) {
    int i = 1;
    printf("PID %d is waiting to be attached\n", getpid());
    while (i) continue;
  }
  LCT_init();
  LCII_log_init();
  // Initialize PMI.
  int num_proc, rank;
  LCT_pmi_initialize();
  rank = LCT_pmi_get_rank();
  num_proc = LCT_pmi_get_size();
  LCI_Assert(num_proc > 0, "PMI ran into an error (num_proc=%d)\n", num_proc);
  LCT_set_rank(rank);
  LCII_pcounters_init();
  // Set some constant from environment variable.
  LCII_env_init(num_proc, rank);
  LCII_papi_init();
  if (LCI_USE_DREG) {
#ifdef LCI_COMPILE_DREG
    LCII_ucs_init();
#else
    LCI_Assert(false, "LCI_COMPILE_DREG is not enabled!\n");
#endif
  }
  // initialize global data structure
  LCIS_server_init(&g_server);
  initialize_packet_heap(&g_heap);
  // UR objects
  LCI_device_init(&LCI_UR_DEVICE);
  LCI_queue_create(LCI_UR_DEVICE, &LCI_UR_CQ);
  LCI_plist_t plist;
  LCI_plist_create(&plist);
  LCI_endpoint_init(&LCI_UR_ENDPOINT, LCI_UR_DEVICE, plist);
  LCI_plist_free(&plist);
  LCI_DBG_Warn(
      "Macro LCI_DEBUG is defined. Running in low-performance debug mode!\n");

  opened = 1;
  LCI_barrier();
  LCI_Log(LCI_LOG_INFO, "device", "LCI_initialize is called\n");
  return LCI_OK;
}

LCI_error_t LCI_initialized(int* flag)
{
  *flag = opened;
  return LCI_OK;
}

LCI_error_t LCI_finalize()
{
  LCI_Log(LCI_LOG_INFO, "device", "LCI_finalize is called\n");
  LCI_barrier();
  LCII_papi_fina();
  LCI_endpoint_free(&LCI_UR_ENDPOINT);
  LCI_queue_free(&LCI_UR_CQ);
  LCI_device_free(&LCI_UR_DEVICE);
  LCIS_server_fina(g_server);
  finalize_packet_heap(&g_heap);
  if (LCI_USE_DREG) {
#ifdef LCI_COMPILE_DREG
    LCII_ucs_cleanup();
#endif
  }
  LCT_pmi_finalize();
  LCII_pcounters_fina();
  LCII_log_fina();
  LCT_fina();

  opened = 0;
  return LCI_OK;
}

// This function is not thread-safe.
LCI_error_t LCII_barrier()
{
  if (LCI_NUM_PROCESSES <= 1) return LCI_OK;

  static LCI_tag_t next_tag = 0;
  static LCI_endpoint_t ep = NULL;
  if (ep == NULL) {
    LCI_plist_t plist;
    LCI_plist_create(&plist);
    LCI_plist_set_comp_type(plist, LCI_PORT_COMMAND, LCI_COMPLETION_SYNC);
    LCI_plist_set_comp_type(plist, LCI_PORT_MESSAGE, LCI_COMPLETION_SYNC);
    LCII_endpoint_init(&ep, LCI_UR_DEVICE, plist, false);
    LCI_plist_free(&plist);
    LCT_pmi_barrier();
  }
  LCI_tag_t tag = next_tag++;
  LCI_Log(LCI_LOG_INFO, "coll", "Start barrier (%d, %p).\n", tag, ep);
  LCI_mbuffer_t buffer;
  int nonsense;
  buffer.address = &nonsense;
  buffer.length = sizeof(nonsense);

  if (LCI_RANK != 0) {
    // Other ranks
    // Phase 1: all the other ranks send a message to rank 0.
    while (LCI_sendm(ep, buffer, 0, tag) != LCI_OK) {
      LCI_progress(LCI_UR_DEVICE);
    }
    // Phase 2: rank 0 send a message to all the other ranks.
    LCI_comp_t sync;
    LCI_sync_create(LCI_UR_DEVICE, 1, &sync);
    LCI_recvm(ep, buffer, 0, tag, sync, NULL);
    while (LCI_sync_test(sync, NULL) != LCI_OK) {
      LCI_progress(LCI_UR_DEVICE);
    }
    LCI_sync_free(&sync);
  } else {
    // rank 0
    // Phase 1: all the other ranks send a message to rank 0.
    LCI_comp_t sync;
    LCI_sync_create(LCI_UR_DEVICE, LCI_NUM_PROCESSES - 1, &sync);
    for (int i = 1; i < LCI_NUM_PROCESSES; ++i) {
      LCI_recvm(ep, buffer, i, tag, sync, NULL);
    }
    while (LCI_sync_test(sync, NULL) != LCI_OK) {
      LCI_progress(LCI_UR_DEVICE);
    }
    // Phase 2: rank 0 send a message to all the other ranks.
    for (int i = 1; i < LCI_NUM_PROCESSES; ++i) {
      while (LCI_sendmc(ep, buffer, i, tag, sync, NULL) != LCI_OK)
        LCI_progress(LCI_UR_DEVICE);
    }
    while (LCI_sync_test(sync, NULL) != LCI_OK) {
      LCI_progress(LCI_UR_DEVICE);
    }
    LCI_sync_free(&sync);
  }
  LCI_Log(LCI_LOG_INFO, "coll", "End barrier (%d, %p).\n", tag, ep);
  return LCI_OK;
}

LCI_error_t LCI_barrier()
{
  //  LCT_pmi_barrier();
  //  return LCI_OK;
  return LCII_barrier();
}