#ifndef SERVER_OFI_H_
#define SERVER_OFI_H_

#include <rdma/fabric.h>
#include <rdma/fi_domain.h>
#include <rdma/fi_endpoint.h>
#include <rdma/fi_tagged.h>
#include <stdlib.h>
#include <string.h>

#include <rdma/fi_cm.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_rma.h>

#define FI_SAFECALL(x)                                                    \
  {                                                                       \
    int err = (x);                                                        \
    if (err < 0) err = -err;                                              \
    if (err) {                                                            \
      LCI_Assert(false, "err : %s (%s:%d)\n", fi_strerror(err), __FILE__, \
                 __LINE__);                                               \
    }                                                                     \
  }                                                                       \
  while (0)                                                               \
    ;

#define LCISI_OFI_CS_TRY_ENTER(endpoint_p, mode, ret)                        \
  if (LCI_BACKEND_TRY_LOCK_MODE & mode && !endpoint_p->is_single_threaded && \
      !LCIU_try_acquire_spinlock(&endpoint_p->super.lock))                   \
    return ret;

#define LCISI_OFI_CS_EXIT(endpoint_p, mode)                                \
  if (LCI_BACKEND_TRY_LOCK_MODE & mode && !endpoint_p->is_single_threaded) \
    LCIU_release_spinlock(&endpoint_p->super.lock);

struct LCISI_endpoint_t;

typedef struct __attribute__((aligned(LCI_CACHE_LINE))) LCISI_server_t {
  struct fi_info* info;
  struct fid_fabric* fabric;
} LCISI_server_t;

typedef struct __attribute__((aligned(LCI_CACHE_LINE))) LCISI_endpoint_t {
  struct LCISI_endpoint_super_t super;
  LCISI_server_t* server;
  struct fid_domain* domain;
  struct fid_ep* ep;
  struct fid_cq* cq;
  struct fid_av* av;
  fi_addr_t* peer_addrs;
  bool is_single_threaded;
} LCISI_endpoint_t;

extern int g_next_rdma_key;

static inline void* LCISI_real_server_reg(LCIS_endpoint_t endpoint_pp,
                                          void* buf, size_t size)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCISI_server_t* server = endpoint_p->server;
  int rdma_key;
  if (server->info->domain_attr->mr_mode & FI_MR_PROV_KEY) {
    rdma_key = 0;
  } else {
    rdma_key = __sync_fetch_and_add(&g_next_rdma_key, 1);
  }
  struct fid_mr* mr;
  FI_SAFECALL(fi_mr_reg(endpoint_p->domain, buf, size,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, rdma_key, 0,
                        &mr, 0));
  if (server->info->domain_attr->mr_mode & FI_MR_ENDPOINT) {
    FI_SAFECALL(fi_mr_bind(mr, &endpoint_p->ep->fid, 0));
    FI_SAFECALL(fi_mr_enable(mr));
  }
  return (void*)mr;
}

static inline void LCISI_real_server_dereg(void* mr_opaque)
{
  struct fid_mr* mr = (struct fid_mr*)mr_opaque;
  FI_SAFECALL(fi_close((struct fid*)&mr->fid));
}

static inline LCIS_mr_t LCISD_rma_reg(LCIS_endpoint_t endpoint_pp, void* buf,
                                      size_t size)
{
  LCIS_mr_t mr;
  mr.mr_p = LCISI_real_server_reg(endpoint_pp, buf, size);
  mr.address = buf;
  mr.length = size;
  return mr;
}

static inline void LCISD_rma_dereg(LCIS_mr_t mr)
{
  LCISI_real_server_dereg(mr.mr_p);
}

static inline LCIS_rkey_t LCISD_rma_rkey(LCIS_mr_t mr)
{
  return fi_mr_key((struct fid_mr*)(mr.mr_p));
}

static inline void* ofi_rma_lkey(LCIS_mr_t mr)
{
  return fi_mr_desc((struct fid_mr*)mr.mr_p);
}

static inline int LCISD_poll_cq(LCIS_endpoint_t endpoint_pp,
                                LCIS_cq_entry_t* entry)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  struct fi_cq_data_entry fi_entry[LCI_CQ_MAX_POLL];
  struct fi_cq_err_entry error;
  char err_data[64];
  ssize_t ne;
  int ret;

  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_POLL, 0)
  ne = fi_cq_read(endpoint_p->cq, &fi_entry, LCI_CQ_MAX_POLL);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_POLL)
  ret = ne;
  LCII_PCOUNTER_ADD(net_poll_cq_calls, 1);
  if (ne > 0) {
    LCII_PCOUNTER_ADD(net_poll_cq_entry_count, ne);
    // Got an entry here
    for (int i = 0; i < ne; i++) {
      if (fi_entry[i].flags & FI_RECV) {
        entry[i].opcode = LCII_OP_RECV;
        entry[i].ctx = fi_entry[i].op_context;
        entry[i].length = fi_entry[i].len;
        entry[i].imm_data = fi_entry[i].data & ((1ULL << 32) - 1);
        entry[i].rank = (int)(fi_entry[i].data >> 32);
      } else if (fi_entry[i].flags & FI_REMOTE_WRITE) {
        entry[i].opcode = LCII_OP_RDMA_WRITE;
        entry[i].ctx = NULL;
        entry[i].imm_data = fi_entry[i].data;
      } else {
        LCI_DBG_Assert(
            fi_entry[i].flags & FI_SEND || fi_entry[i].flags & FI_WRITE,
            "Unexpected OFI opcode!\n");
        entry[i].opcode = LCII_OP_SEND;
        entry[i].ctx = fi_entry[i].op_context;
      }
    }
  } else if (ne == -FI_EAGAIN) {
    ret = 0;
  } else {
    LCI_Assert(ne == -FI_EAVAIL, "unexpected return error: %s\n",
               fi_strerror(-ne));
    error.err_data = err_data;
    error.err_data_size = sizeof(err_data);
    ssize_t ret_cqerr = fi_cq_readerr(endpoint_p->cq, &error, 0);
    // The error was already consumed, most likely by another thread,
    if (ret_cqerr == -FI_EAGAIN) return 0;
    //    LCI_Warn("Err %d: %s\n", error.err, fi_strerror(error.err));
    LCI_Assert(false, "Err %d: %s\n", error.err, fi_strerror(error.err));
  }
  return ret;
}

static inline LCI_error_t LCISD_post_recv(LCIS_endpoint_t endpoint_pp,
                                          void* buf, uint32_t size,
                                          LCIS_mr_t mr, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;

  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_RECV,
                         LCI_ERR_RETRY_LOCK)
  ssize_t ret =
      fi_recv(endpoint_p->ep, buf, size, ofi_rma_lkey(mr), FI_ADDR_UNSPEC, ctx);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_RECV)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY;
  else {
    FI_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_sends(LCIS_endpoint_t endpoint_pp,
                                           int rank, void* buf, size_t size,
                                           LCIS_meta_t meta)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  ssize_t ret =
      fi_injectdata(endpoint_p->ep, buf, size, (uint64_t)LCI_RANK << 32 | meta,
                    endpoint_p->peer_addrs[rank]);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_send(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size, LCIS_mr_t mr,
                                          LCIS_meta_t meta, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  ssize_t ret =
      fi_senddata(endpoint_p->ep, buf, size, ofi_rma_lkey(mr),
                  (uint64_t)LCI_RANK << 32 | meta, endpoint_p->peer_addrs[rank],
                  (struct fi_context*)ctx);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_puts(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size,
                                          uintptr_t base, LCIS_offset_t offset,
                                          LCIS_rkey_t rkey)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  uintptr_t addr;
  if (endpoint_p->server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR ||
      endpoint_p->server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  struct fi_msg_rma msg;
  struct iovec iov;
  struct fi_rma_iov riov;
  iov.iov_base = buf;
  iov.iov_len = size;
  msg.msg_iov = &iov;
  msg.desc = NULL;
  msg.iov_count = 1;
  msg.addr = endpoint_p->peer_addrs[rank];
  riov.addr = addr;
  riov.len = size;
  riov.key = rkey;
  msg.rma_iov = &riov;
  msg.rma_iov_count = 1;
  msg.context = NULL;
  msg.data = 0;
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  //  ssize_t ret = fi_inject_write(endpoint_p->ep, buf, size,
  //                                endpoint_p->peer_addrs[rank], addr, rkey);
  ssize_t ret =
      fi_writemsg(endpoint_p->ep, &msg, FI_INJECT | FI_DELIVERY_COMPLETE);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t LCISD_post_put(LCIS_endpoint_t endpoint_pp, int rank,
                                         void* buf, size_t size, LCIS_mr_t mr,
                                         uintptr_t base, LCIS_offset_t offset,
                                         LCIS_rkey_t rkey, void* ctx)
{
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  uintptr_t addr;
  if (endpoint_p->server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR ||
      endpoint_p->server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  struct fi_msg_rma msg;
  struct iovec iov;
  struct fi_rma_iov riov;
  void* desc = ofi_rma_lkey(mr);
  iov.iov_base = buf;
  iov.iov_len = size;
  msg.msg_iov = &iov;
  msg.desc = &desc;
  msg.iov_count = 1;
  msg.addr = endpoint_p->peer_addrs[rank];
  riov.addr = addr;
  riov.len = size;
  riov.key = rkey;
  msg.rma_iov = &riov;
  msg.rma_iov_count = 1;
  msg.context = ctx;
  msg.data = 0;
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  //  ssize_t ret = fi_write(endpoint_p->ep, buf, size, ofi_rma_lkey(mr),
  //                         endpoint_p->peer_addrs[rank], addr, rkey, ctx);
  ssize_t ret = fi_writemsg(endpoint_p->ep, &msg, FI_DELIVERY_COMPLETE);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
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
  uintptr_t addr;
  if (endpoint_p->server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR ||
      endpoint_p->server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  ssize_t ret = fi_inject_writedata(endpoint_p->ep, buf, size, meta,
                                    endpoint_p->peer_addrs[rank], addr, rkey);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
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
  uintptr_t addr;
  if (endpoint_p->server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR ||
      endpoint_p->server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  LCISI_OFI_CS_TRY_ENTER(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND,
                         LCI_ERR_RETRY_LOCK)
  ssize_t ret = fi_writedata(endpoint_p->ep, buf, size, ofi_rma_lkey(mr), meta,
                             endpoint_p->peer_addrs[rank], addr, rkey, ctx);
  LCISI_OFI_CS_EXIT(endpoint_p, LCI_BACKEND_TRY_LOCK_SEND)
  if (ret == FI_SUCCESS)
    return LCI_OK;
  else if (ret == -FI_EAGAIN)
    return LCI_ERR_RETRY_NOMEM;
  else {
    FI_SAFECALL(ret);
    return LCI_ERR_FATAL;
  }
}

#endif
