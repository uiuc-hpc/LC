#ifndef SERVER_OFI_H_
#define SERVER_OFI_H_

#include <rdma/fabric.h>
#include <rdma/fi_domain.h>
#include <rdma/fi_endpoint.h>
#include <rdma/fi_tagged.h>
#include <stdlib.h>
#include <string.h>
#ifdef LCI_USE_DREG
#include "dreg.h"
#endif

#include <rdma/fi_cm.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_rma.h>

#define FI_SAFECALL(x)                                                    \
  {                                                                       \
    int err = (x);                                                        \
    if (err < 0) err = -err;                                              \
    if (err) {                                                            \
      LCM_Assert(false, "err : %s (%s:%d)\n",                             \
              fi_strerror(err), __FILE__, __LINE__);                      \
    }                                                                     \
  }                                                                       \
  while (0)                                                               \
    ;

typedef struct LCISI_server_t {
  LCI_device_t device;
  struct fi_info* info;
  struct fid_fabric* fabric;
  struct fid_domain* domain;
  struct fid_ep* ep;
  struct fid_cq* cq;
  struct fid_av* av;
  fi_addr_t *peer_addrs;
  void *heap_desc;
} LCISI_server_t __attribute__((aligned(64)));

extern int g_next_rdma_key;

static inline uintptr_t _real_server_reg(LCIS_server_t s, void* buf, size_t size)
{
  LCISI_server_t *server = (LCISI_server_t*) s;
  struct fid_mr* mr;
  int rdma_key = __sync_fetch_and_add(&g_next_rdma_key, 1);
  FI_SAFECALL(fi_mr_reg(server->domain, buf, size,
                        FI_READ | FI_WRITE | FI_REMOTE_WRITE, 0, rdma_key, 0,
                        &mr, 0));
  LCM_DBG_Log(LCM_LOG_DEBUG, "real mem reg: mr %p buf %p size %lu lkey %p rkey %lu\n",
              mr, buf, size, fi_mr_desc(mr), fi_mr_key(mr));
  return (uintptr_t)mr;
}

static inline void _real_server_dereg(uintptr_t mr_opaque)
{
  struct fid_mr *mr = (struct fid_mr*) mr_opaque;
  LCM_DBG_Log(LCM_LOG_DEBUG, "real mem dereg: mr %p lkey %p rkey %lu\n",
              mr, fi_mr_desc(mr), fi_mr_key(mr));
  FI_SAFECALL(fi_close((struct fid*) &mr->fid));
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
  return fi_mr_key((struct fid_mr*)(((dreg_entry*)mr.mr_p)->memhandle[0]));
}

static inline void* ofi_rma_lkey(LCIS_mr_t mr)
{
  return fi_mr_desc((struct fid_mr*)(((dreg_entry*)mr.mr_p)->memhandle[0]));
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
  return fi_mr_key((struct fid_mr*)(mr.mr_p));
}

static inline void* ofi_rma_lkey(LCIS_mr_t mr)
{
  return fi_mr_desc((struct fid_mr*)mr.mr_p);
}

#endif

static inline int LCID_poll_cq(LCIS_server_t s, LCIS_cq_entry_t*entry) {
  LCISI_server_t *server = (LCISI_server_t*) s;
  struct fi_cq_tagged_entry fi_entry[LCI_CQ_MAX_POLL];
  struct fi_cq_err_entry error;
  ssize_t ne;
  int ret;

  ne = fi_cq_read(server->cq, &fi_entry, LCI_CQ_MAX_POLL);
  ret = ne;
  if (ne > 0) {
    // Got an entry here
    for (int i = 0; i < ne; i++) {
      if (fi_entry[i].flags & FI_RECV) {
        // we use tag to pass src_rank, because it is hard to get src_rank
        // from fi_addr_t. TODO: Need to improve
        entry[i].opcode = LCII_OP_RECV;
        entry[i].ctx = fi_entry[i].op_context;
        entry[i].length = fi_entry[i].len;
        entry[i].imm_data = fi_entry[i].data;
        entry[i].rank = fi_entry[i].tag;
      } else if (fi_entry[i].flags & FI_REMOTE_WRITE) {
        entry[i].opcode = LCII_OP_RDMA_WRITE;
        entry[i].ctx = NULL;
        entry[i].imm_data = fi_entry[i].data;
      } else {
        LCM_DBG_Assert(fi_entry[i].flags & FI_SEND ||
                       fi_entry[i].flags & FI_WRITE,
                       "Unexpected OFI opcode!\n");
        entry[i].opcode = LCII_OP_SEND;
        entry[i].ctx = fi_entry[i].op_context;
      }
    }
  } else if (ne == -FI_EAGAIN) {
    ret = 0;
  } else {
    LCM_DBG_Assert(ne == -FI_EAVAIL, "unexpected return error: %s\n", fi_strerror(-ne));
    fi_cq_readerr(server->cq, &error, 0);
    LCM_Assert(false, "Err %d: %s\n", error.err, fi_strerror(error.err));
  }
  return ret;
}

static inline void lc_server_post_recv(LCIS_server_t s, void *buf, uint32_t size, LCIS_mr_t mr, void *ctx) {
  LCISI_server_t *server = (LCISI_server_t*) s;
  FI_SAFECALL(fi_trecv(server->ep, buf, size, ofi_rma_lkey(mr),
                       FI_ADDR_UNSPEC, /*any*/0, ~(uint64_t)0 /*ignore all*/,
                       ctx));
}

static inline LCI_error_t lc_server_sends(LCIS_server_t s, int rank, void* buf,
                                          size_t size, LCIS_meta_t meta)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post sends: rank %d buf %p size %lu meta %d\n",
              rank, buf, size, meta);
  LCISI_server_t *server = (LCISI_server_t*) s;
  int ret = fi_tinjectdata(server->ep, buf, size, meta, server->peer_addrs[rank],
                           LCI_RANK /*tag*/);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_send(LCIS_server_t s, int rank, void* buf,
                                         size_t size, LCIS_mr_t mr,
                                         LCIS_meta_t meta,
                                         void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post send: rank %d buf %p size %lu lkey %p meta %d ctx %p\n",
              rank, buf, size, ofi_rma_lkey(mr), meta, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;
  int ret = fi_tsenddata(server->ep, buf, size, ofi_rma_lkey(mr),
                         meta, server->peer_addrs[rank], LCI_RANK /*tag*/,
                         (struct fi_context*)ctx);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
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
  uintptr_t addr;
  if (server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR || server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  int ret = fi_inject_write(server->ep, buf, size, server->peer_addrs[rank], addr, rkey);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_put(LCIS_server_t s, int rank, void* buf,
                                        size_t size, LCIS_mr_t mr,
                                        uintptr_t base, uint32_t offset,
                                        LCIS_rkey_t rkey, void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post put: rank %d buf %p size %lu lkey %p base %p "
              "offset %d rkey %lu ctx %p\n", rank, buf,
              size, ofi_rma_lkey(mr), (void*) base, offset, rkey, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;
  uintptr_t addr;
  if (server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR || server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  int ret = fi_write(server->ep, buf, size, ofi_rma_lkey(mr),
                         server->peer_addrs[rank], addr, rkey, ctx);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_putImms(LCIS_server_t s, int rank, void* buf,
                                            size_t size, uintptr_t base,
                                            uint32_t offset, LCIS_rkey_t rkey,
                                            uint32_t meta)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post puts: rank %d buf %p size %lu base %p offset %d "
                             "rkey %lu meta %d\n", rank, buf,
              size, (void*) base, offset, rkey, meta);
  LCISI_server_t *server = (LCISI_server_t*) s;
  uintptr_t addr;
  if (server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR || server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  int ret = fi_inject_writedata(server->ep, buf, size, meta, server->peer_addrs[rank], addr, rkey);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}

static inline LCI_error_t lc_server_putImm(LCIS_server_t s, int rank, void* buf,
                                           size_t size, LCIS_mr_t mr,
                                           uintptr_t base, uint32_t offset,
                                           LCIS_rkey_t rkey, LCIS_meta_t meta,
                                           void* ctx)
{
  LCM_DBG_Log(LCM_LOG_DEBUG, "post put: rank %d buf %p size %lu lkey %p base %p "
                             "offset %d rkey %lu meta %u ctx %p\n", rank, buf,
              size, ofi_rma_lkey(mr), (void*) base, offset, rkey, meta, ctx);
  LCISI_server_t *server = (LCISI_server_t*) s;
  uintptr_t addr;
  if (server->info->domain_attr->mr_mode & FI_MR_VIRT_ADDR || server->info->domain_attr->mr_mode & FI_MR_BASIC) {
    addr = base + offset;
  } else {
    addr = offset;
  }
  int ret = fi_writedata(server->ep, buf, size, ofi_rma_lkey(mr), meta,
                         server->peer_addrs[rank], addr, rkey, ctx);
  if (ret == FI_SUCCESS) return LCI_OK;
  else if (ret == -FI_EAGAIN) return LCI_ERR_RETRY;
  else {
    LCM_Assert(false, "err %d : %s (%s:%d)\n",
               ret, fi_strerror(ret), __FILE__, __LINE__);
    return LCI_ERR_FATAL;
  }
}


#endif
