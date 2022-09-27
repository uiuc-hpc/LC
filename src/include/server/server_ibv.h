#ifndef SERVER_IBV_H_
#define SERVER_IBV_H_

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "infiniband/verbs.h"
#ifdef LCI_USE_DREG
#include "dreg.h"
#endif

#define IBV_SAFECALL(x)                                               \
  {                                                                   \
    int err = (x);                                                    \
    if (err) {                                                        \
      LCM_Assert(false, "err %d : %s (%s:%d)\n",                      \
              err, strerror(err), __FILE__, __LINE__);                \
    }                                                                 \
  }                                                                   \
  while (0)                                                           \
    ;

typedef struct LCISI_server_t {
  LCI_device_t device;

  // Device fields.
  struct ibv_device **dev_list;
  struct ibv_device *ib_dev;
  struct ibv_context* dev_ctx;
  struct ibv_pd* dev_pd;
  struct ibv_device_attr dev_attr;
  struct ibv_port_attr port_attr;
  struct ibv_srq* dev_srq;
  struct ibv_cq* cq;
  uint8_t dev_port;

  // Connections O(N)
  struct ibv_qp** qps;

  // Helper fields.
  int* qp2rank;
  int qp2rank_mod;
  size_t max_inline;
} LCISI_server_t __attribute__((aligned(64)));

static inline uintptr_t _real_server_reg(LCIS_server_t s, void* buf, size_t size)
{
  LCISI_server_t *server = (LCISI_server_t*) s;
  int mr_flags =
      IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE;
  return (uintptr_t)ibv_reg_mr(server->dev_pd, buf, size, mr_flags);
}

static inline void _real_server_dereg(uintptr_t mem)
{
  ibv_dereg_mr((struct ibv_mr*)mem);
}

#ifdef LCI_USE_DREG
static inline LCIS_mr_t lc_server_rma_reg(LCIS_server_t s, void* buf, size_t size)
{
  dreg_entry *entry = dreg_register(s, buf, size);
  LCM_DBG_Assert(entry != NULL, "Unable to register more memory!");
  LCIS_mr_t mr;
  mr.mr_p = (uintptr_t) entry;
  mr.address = (void*) (entry->pagenum << DREG_PAGEBITS);
  mr.length = entry->npages << DREG_PAGEBITS;
  return mr;
}

static inline void lc_server_rma_dereg(LCIS_mr_t mr)
{
  dreg_unregister((dreg_entry*)mr.mr_p);
}

static inline LCIS_rkey_t lc_server_rma_rkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)(((dreg_entry*)mr.mr_p)->memhandle[0]))->rkey;
}

static inline uint32_t ibv_rma_lkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)(((dreg_entry*)mr.mr_p)->memhandle[0]))->lkey;
}

#else

static inline LCIS_mr_t lc_server_rma_reg(LCIS_server_t s, void* buf, size_t size)
{
  LCIS_mr_t mr;
  mr.mr_p = _real_server_reg(s, buf, size);
  mr.address = buf;
  mr.length = size;
  return mr;
}

static inline void lc_server_rma_dereg(LCIS_mr_t mr)
{
  _real_server_dereg(mr.mr_p);
}

static inline LCIS_rkey_t lc_server_rma_rkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)mr.mr_p)->rkey;
}

static inline uint32_t ibv_rma_lkey(LCIS_mr_t mr)
{
  return ((struct ibv_mr*)mr.mr_p)->lkey;
}

#endif

static inline int LCID_poll_cq(LCIS_server_t s, LCIS_cq_entry_t *entry)
{
  LCISI_server_t *server = (LCISI_server_t*) s;
  struct ibv_wc wc[LCI_CQ_MAX_POLL];
  int ne = ibv_poll_cq(server->cq, LCI_CQ_MAX_POLL, wc);
  LCM_DBG_Assert(ne >= 0, "ibv_poll_cq returns error\n");

  for (int i = 0; i < ne; i++) {
    LCM_DBG_Assert(wc[i].status == IBV_WC_SUCCESS,
                   "Failed status %s (%d) for wr_id %d\n",
                   ibv_wc_status_str(wc[i].status), wc[i].status,
                   (int)wc[i].wr_id);
    if (wc[i].opcode == IBV_WC_RECV) {
      entry[i].opcode = LCII_OP_RECV;
      entry[i].ctx = (void*)wc[i].wr_id;
      entry[i].length = wc[i].byte_len;
      entry[i].imm_data = wc[i].imm_data;
      entry[i].rank = server->qp2rank[wc[i].qp_num % server->qp2rank_mod];
    } else if (wc[i].opcode == IBV_WC_RECV_RDMA_WITH_IMM) {
      entry[i].opcode = LCII_OP_RDMA_WRITE;
      entry[i].ctx = (void*) wc[i].wr_id;
      entry[i].imm_data = wc[i].imm_data;
    } else {
      LCM_DBG_Assert(wc[i].opcode == IBV_WC_SEND ||
                     wc[i].opcode == IBV_WC_RDMA_WRITE,
                     "Unexpected IBV opcode!\n");
      entry[i].opcode = LCII_OP_SEND;
      entry[i].ctx = (void*) wc[i].wr_id;
    }
  }
  return ne;
}

static inline void lc_server_post_recv(LCIS_server_t s, void *buf, uint32_t size, LCIS_mr_t mr, void *ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post recv: buf %p size %u lkey %u user_context %p\n",
              buf, size, ibv_rma_lkey(mr), ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;
  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length = size;
  list.lkey	= ibv_rma_lkey(mr);
  struct ibv_recv_wr wr;
  wr.wr_id	    = (uint64_t) ctx;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  struct ibv_recv_wr *bad_wr;
  IBV_SAFECALL(ibv_post_srq_recv(server->dev_srq, &wr, &bad_wr));
}

static inline LCI_error_t lc_server_sends(LCIS_server_t s, int rank, void* buf,
                                          size_t size, LCIS_meta_t meta)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post sends: rank %d buf %p size %lu meta %d\n",
              rank, buf, size, meta);
  LCISI_server_t *server = (LCISI_server_t*) s;
  LCM_DBG_Assert(size <= server->max_inline, "%lu exceed the inline message size"
                                             "limit! %lu\n", size, server->max_inline);

  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= 0;
  struct ibv_send_wr wr;
  wr.wr_id	= 0;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_SEND_WITH_IMM;
  wr.send_flags = IBV_SEND_INLINE;
  wr.imm_data   = meta;

//  static int ninline = 0;
//  int ninline_old = __sync_fetch_and_add(&ninline, 1);
//  if (ninline_old == 63) {
    wr.send_flags |= IBV_SEND_SIGNALED;
//    ninline = 0;
//  }

  struct ibv_send_wr *bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_send(LCIS_server_t s, int rank, void* buf,
                                         size_t size, LCIS_mr_t mr, LCIS_meta_t meta,
                                         void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post send: rank %d buf %p size %lu lkey %d meta %d ctx %p\n",
              rank, buf, size, ibv_rma_lkey(mr), meta, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;

  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= ibv_rma_lkey(mr);
  struct ibv_send_wr wr;
  wr.wr_id	= (uintptr_t) ctx;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_SEND_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.imm_data   = meta;
  if (size <= server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }

  struct ibv_send_wr* bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_puts(LCIS_server_t s, int rank, void* buf,
                                         size_t size, uintptr_t base,
                                         uint32_t offset, LCIS_rkey_t rkey)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post puts: rank %d buf %p size %lu base %p offset %d "
              "rkey %lu\n", rank, buf,
              size, (void*) base, offset, rkey);
  LCISI_server_t *server = (LCISI_server_t*) s;
  LCM_DBG_Assert(size <= server->max_inline, "%lu exceed the inline message size"
                 "limit! %lu\n", size, server->max_inline);
  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= 0;
  struct ibv_send_wr wr;
  wr.wr_id	= 0;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_RDMA_WRITE;
  wr.send_flags = IBV_SEND_SIGNALED | IBV_SEND_INLINE;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;

  struct ibv_send_wr *bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_put(LCIS_server_t s, int rank, void* buf,
                                        size_t size, LCIS_mr_t mr, uintptr_t base,
                                        uint32_t offset, LCIS_rkey_t rkey,
                                        void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post put: rank %d buf %p size %lu lkey %u base %p "
              "offset %d rkey %lu ctx %p\n", rank, buf,
              size, ibv_rma_lkey(mr), (void*) base, offset, rkey, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;

  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= ibv_rma_lkey(mr);
  struct ibv_send_wr wr;
  wr.wr_id      = (uint64_t) ctx;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_RDMA_WRITE;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  if (size <= server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }
  struct ibv_send_wr *bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_putImms(LCIS_server_t s, int rank, void* buf,
                                         size_t size, uintptr_t base, uint32_t offset,
                                         LCIS_rkey_t rkey, uint32_t meta)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post puts: rank %d buf %p size %lu base %p offset %d "
              "rkey %lu meta %d\n", rank, buf,
              size, (void*) base, offset, rkey, meta);
  LCISI_server_t *server = (LCISI_server_t*) s;
  LCM_DBG_Assert(size <= server->max_inline, "%lu exceed the inline message size"
                                             "limit! %lu\n", size, server->max_inline);
  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= 0;
  struct ibv_send_wr wr;
  wr.wr_id	= 0;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_RDMA_WRITE_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED | IBV_SEND_INLINE;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  wr.imm_data   = meta;

  struct ibv_send_wr *bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_putImm(LCIS_server_t s, int rank, void* buf,
                                        size_t size, LCIS_mr_t mr, uintptr_t base,
                                        uint32_t offset, LCIS_rkey_t rkey,
                                        LCIS_meta_t meta, void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post put: rank %d buf %p size %lu lkey %u base %p "
              "offset %d rkey %lu meta %u ctx %p\n", rank, buf,
              size, ibv_rma_lkey(mr), (void*) base, offset, rkey, meta, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;

  struct ibv_sge list;
  list.addr	= (uint64_t) buf;
  list.length   = size;
  list.lkey	= ibv_rma_lkey(mr);
  struct ibv_send_wr wr;
  wr.wr_id      = (uint64_t) ctx;
  wr.next       = NULL;
  wr.sg_list    = &list;
  wr.num_sge    = 1;
  wr.opcode     = IBV_WR_RDMA_WRITE_WITH_IMM;
  wr.send_flags = IBV_SEND_SIGNALED;
  wr.imm_data   = meta;
  wr.wr.rdma.remote_addr = (uintptr_t)(base + offset);
  wr.wr.rdma.rkey = rkey;
  if (size <= server->max_inline) {
    wr.send_flags |= IBV_SEND_INLINE;
  }
  struct ibv_send_wr *bad_wr;
  int ret = ibv_post_send(server->qps[rank], &wr, &bad_wr);
  if (ret == 0) return LCI_OK;
  else if (ret == ENOMEM) return LCI_ERR_RETRY; // exceed send queue capacity
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

#endif
