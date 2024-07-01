#ifndef SERVER_IBV_H_
#define SERVER_IBV_H_

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "infiniband/verbs.h"

#define IBV_SAFECALL(x)                                                  \
  {                                                                      \
    int err = (x);                                                       \
    if (err) {                                                           \
      LCI_DBG_Assert(false, "err %d : %s (%s:%d)\n", err, strerror(err), \
                     __FILE__, __LINE__);                                \
    }                                                                    \
  }                                                                      \
  while (0)                                                              \
    ;

typedef struct __attribute__((aligned(LCI_CACHE_LINE))) LCISI_server_t {
  // Device fields.
  struct ibv_device** dev_list;
  struct ibv_device* ib_dev;
  struct ibv_context* dev_ctx;
  struct ibv_pd* dev_pd;
  struct ibv_device_attr dev_attr;
  struct ibv_device_attr_ex dev_attrx;
  struct ibv_port_attr port_attr;
  uint8_t dev_port;
  struct ibv_mr* odp_mr;
  size_t max_inline;
  // event polling thread
  pthread_t event_polling_thread;
  atomic_bool event_polling_thread_run;
} LCISI_server_t;

struct __attribute__((aligned(LCI_CACHE_LINE))) LCISI_ibv_qp_extra_t {
  LCIU_spinlock_t lock;
  struct ibv_td* td;
  struct ibv_pd* pd;
  LCIU_CACHE_PADDING(sizeof(LCIU_spinlock_t) + sizeof(struct ibv_td*) +
                     sizeof(struct ibv_pd*));
};

typedef struct LCISI_endpoint_t {
  struct LCISI_endpoint_super_t super;
  struct LCISI_server_t* server;
  // Connections O(N)
  struct ibv_td* td;
  struct ibv_pd* pd;
  struct ibv_qp** qps;
  struct LCISI_ibv_qp_extra_t* qp_extras;
  struct ibv_cq* cq;
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  LCIU_spinlock_t cq_lock;
#endif
  struct ibv_srq* srq;
  // Helper fields.
  int* qp2rank;
  int qp2rank_mod;
} LCISI_endpoint_t;

static inline void* LCISI_real_server_reg(LCIS_endpoint_t endpoint_pp,
                                          void* buf, size_t size)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCISI_server_t* server = endpoint_p->server;
  int mr_flags;
  if (LCI_IBV_USE_ODP) {
    mr_flags = IBV_ACCESS_ON_DEMAND | IBV_ACCESS_LOCAL_WRITE |
               IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE;
  } else {
    mr_flags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ |
               IBV_ACCESS_REMOTE_WRITE;
  }
  return (void*)ibv_reg_mr(server->dev_pd, buf, size, mr_flags);
}

static inline void LCISI_real_server_dereg(void* mem)
{
  ibv_dereg_mr((struct ibv_mr*)mem);
}

static inline uint32_t ibv_rma_lkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)mr.mr_p)->lkey;
}

static inline LCIS_mr_t LCISD_rma_reg(LCIS_endpoint_t endpoint_pp, void* buf,
                                      size_t size)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCISI_server_t* server = endpoint_p->server;
  LCIS_mr_t mr;
  if (LCI_IBV_USE_ODP == 2) {
    mr.mr_p = server->odp_mr;
    mr.address = buf;
    mr.length = size;
  } else {
    mr.mr_p = LCISI_real_server_reg(endpoint_pp, buf, size);
    mr.address = buf;
    mr.length = size;
  }
  if (LCI_IBV_USE_PREFETCH) {
    struct ibv_sge list = {
        .addr = (uintptr_t)buf,
        .length = (uint32_t)size,
        .lkey = ibv_rma_lkey(mr),
    };
    IBV_SAFECALL(ibv_advise_mr(
        server->dev_pd, IBV_ADVISE_MR_ADVICE_PREFETCH_WRITE, 0, &list, 1));
  }
  return mr;
}

static inline void LCISD_rma_dereg(LCIS_mr_t mr)
{
  if (LCI_IBV_USE_ODP == 2) {
    // do nothing
  } else {
    LCISI_real_server_dereg(mr.mr_p);
  }
}

static inline LCIS_rkey_t LCISD_rma_rkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)mr.mr_p)->rkey;
}

static inline int LCISD_poll_cq(LCIS_endpoint_t endpoint_pp,
                                LCIS_cq_entry_t* entry)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  struct ibv_wc wc[LCI_CQ_MAX_POLL];
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  if (!LCIU_try_acquire_spinlock(&endpoint_p->cq_lock)) return 0;
#endif
  int ne = ibv_poll_cq(endpoint_p->cq, LCI_CQ_MAX_POLL, wc);
  LCI_DBG_Assert(ne >= 0, "ibv_poll_cq returns error %d\n", ne);
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  LCIU_release_spinlock(&endpoint_p->cq_lock);
#endif
  LCII_PCOUNTER_ADD(net_poll_cq_calls, 1);
  if (ne > 0) LCII_PCOUNTER_ADD(net_poll_cq_entry_count, ne);
  for (int i = 0; i < ne; i++) {
    LCI_DBG_Assert(
        wc[i].status == IBV_WC_SUCCESS, "Failed status %s (%d) for wr_id %d\n",
        ibv_wc_status_str(wc[i].status), wc[i].status, (int)wc[i].wr_id);
    if (wc[i].opcode == IBV_WC_RECV) {
      entry[i].opcode = LCII_OP_RECV;
      entry[i].ctx = (void*)wc[i].wr_id;
      entry[i].length = wc[i].byte_len;
      entry[i].imm_data = wc[i].imm_data;
      entry[i].rank =
          endpoint_p->qp2rank[wc[i].qp_num % endpoint_p->qp2rank_mod];
    } else if (wc[i].opcode == IBV_WC_RECV_RDMA_WITH_IMM) {
      entry[i].opcode = LCII_OP_RDMA_WRITE;
      entry[i].ctx = (void*)wc[i].wr_id;
      entry[i].imm_data = wc[i].imm_data;
    } else {
      LCI_DBG_Assert(
          wc[i].opcode == IBV_WC_SEND || wc[i].opcode == IBV_WC_RDMA_WRITE,
          "Unexpected IBV opcode!\n");
      entry[i].opcode = LCII_OP_SEND;
      entry[i].ctx = (void*)wc[i].wr_id;
    }
  }
  return ne;
}

static inline LCI_error_t LCISD_post_recv(LCIS_endpoint_t endpoint_pp,
                                          void* buf, uint32_t size,
                                          LCIS_mr_t mr, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  struct ibv_sge list;
  list.addr = (uint64_t)buf;
  list.length = size;
  list.lkey = ibv_rma_lkey(mr);
  struct ibv_recv_wr wr;
  wr.wr_id = (uint64_t)ctx;
  wr.next = NULL;
  wr.sg_list = &list;
  wr.num_sge = 1;
  struct ibv_recv_wr* bad_wr;
  IBV_SAFECALL(ibv_post_srq_recv(endpoint_p->srq, &wr, &bad_wr));
  return LCI_OK;
}

static inline LCI_error_t LCISD_post_sends(LCIS_endpoint_t endpoint_pp,
                                           int rank, void* buf, size_t size,
                                           LCIS_meta_t meta)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCI_DBG_Assert(size <= endpoint_p->server->max_inline,
                 "%lu exceed the inline message size\n"
                 "limit! %lu\n",
                 size, endpoint_p->server->max_inline);
  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = 0;
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = 0;
  wr.next = NULL;
  wr.opcode = IBV_WR_SEND_WITH_IMM;
  wr.send_flags = IBV_SEND_INLINE;
  wr.imm_data = meta;

  //  static int ninline = 0;
  //  int ninline_old = __sync_fetch_and_add(&ninline, 1);
  //  if (ninline_old == 63) {
  wr.send_flags |= IBV_SEND_SIGNALED;
  //    ninline = 0;
  //  }

  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_send(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size, LCIS_mr_t mr,
                                          LCIS_meta_t meta, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;

  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = ibv_rma_lkey(mr);
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = (uintptr_t)ctx;
  wr.next = NULL;
  wr.opcode = IBV_WR_SEND_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.imm_data = meta;
  if (size <= endpoint_p->server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }

  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_puts(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size,
                                          uintptr_t base, LCIS_offset_t offset,
                                          LCIS_rkey_t rkey)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCI_DBG_Assert(size <= endpoint_p->server->max_inline,
                 "%lu exceed the inline message size\n"
                 "limit! %lu\n",
                 size, endpoint_p->server->max_inline);

  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = 0;
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = 0;
  wr.next = NULL;
  wr.opcode = IBV_WR_RDMA_WRITE;
  wr.send_flags = IBV_SEND_SIGNALED | IBV_SEND_INLINE;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;

  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_put(LCIS_endpoint_t endpoint_pp, int rank,
                                         void* buf, size_t size, LCIS_mr_t mr,
                                         uintptr_t base, LCIS_offset_t offset,
                                         LCIS_rkey_t rkey, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;

  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = ibv_rma_lkey(mr);
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = (uint64_t)ctx;
  wr.next = NULL;
  wr.opcode = IBV_WR_RDMA_WRITE;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  if (size <= endpoint_p->server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }
  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_putImms(LCIS_endpoint_t endpoint_pp,
                                             int rank, void* buf, size_t size,
                                             uintptr_t base,
                                             LCIS_offset_t offset,
                                             LCIS_rkey_t rkey, uint32_t meta)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCI_DBG_Assert(size <= endpoint_p->server->max_inline,
                 "%lu exceed the inline message size\n"
                 "limit! %lu\n",
                 size, endpoint_p->server->max_inline);
  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = 0;
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = 0;
  wr.next = NULL;
  wr.opcode = IBV_WR_RDMA_WRITE_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED | IBV_SEND_INLINE;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  wr.imm_data = meta;

  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_putImm(LCIS_endpoint_t endpoint_pp,
                                            int rank, void* buf, size_t size,
                                            LCIS_mr_t mr, uintptr_t base,
                                            LCIS_offset_t offset,
                                            LCIS_rkey_t rkey, LCIS_meta_t meta,
                                            void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;

  struct ibv_sge list;
  struct ibv_send_wr wr;
  if (LCT_likely(size > 0)) {
    list.addr = (uint64_t)buf;
    list.length = size;
    list.lkey = ibv_rma_lkey(mr);
    wr.sg_list = &list;
    wr.num_sge = 1;
  } else {
    // With mlx4 backend, sge.length = 0 will be treated as 0x80000000.
    // With mlx5 backend, it will just be 0.
    // So we better just set num_sge to 0 here.
    wr.sg_list = NULL;
    wr.num_sge = 0;
  }
  wr.wr_id = (uint64_t)ctx;
  wr.next = NULL;
  wr.opcode = IBV_WR_RDMA_WRITE_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.imm_data = meta;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  if (size <= endpoint_p->server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }
  struct ibv_send_wr* bad_wr;
  if (endpoint_p->qp_extras &&
      !LCIU_try_acquire_spinlock(&endpoint_p->qp_extras[rank].lock))
    return LCI_ERR_RETRY_LOCK;
  int ret = ibv_post_send(endpoint_p->qps[rank], &wr, &bad_wr);
  if (endpoint_p->qp_extras)
    LCIU_release_spinlock(&endpoint_p->qp_extras[rank].lock);
  if (ret == 0)
    return LCI_OK;
  else if (ret == ENOMEM)
    return LCI_ERR_RETRY_NOMEM;  // exceed send queue capacity
  else {
    IBV_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

#endif
