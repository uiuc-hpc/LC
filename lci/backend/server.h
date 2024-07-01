#ifndef SERVER_H_
#define SERVER_H_

struct LCID_server_opaque_t;
typedef struct LCID_server_opaque_t* LCIS_server_t;

struct LCID_endpoint_opaque_t;
typedef struct LCID_endpoint_opaque_t* LCIS_endpoint_t;
struct LCISI_endpoint_super_t {
  LCIU_spinlock_t lock;
};

typedef uint64_t LCIS_offset_t;

typedef struct LCIS_mr_t {
  // server-specific mr object
  void* mr_p;
  void* address;
  size_t length;
} LCIS_mr_t;

#ifdef LCI_USE_SERVER_UCX
typedef struct {
  uint64_t val[2];
} LCIS_rkey_t;
#else
typedef uint64_t LCIS_rkey_t;
#endif
typedef uint32_t LCIS_meta_t;  // immediate data
enum LCIS_opcode_t {
  LCII_OP_SEND,
  LCII_OP_RECV,
  LCII_OP_RDMA_WRITE,
};
typedef struct LCIS_cq_entry_t {
  enum LCIS_opcode_t opcode;
  int rank;
  void* ctx;
  size_t length;
  uint32_t imm_data;
} LCIS_cq_entry_t;

/* Following functions are required to be implemented by LCI */

static inline void LCIS_serve_recv(void* p, int rank, size_t length,
                                   uint32_t imm_data);
static inline void LCIS_serve_rdma(uint32_t imm_data);
static inline void LCIS_serve_send(void* ctx);

/* Following functions are required to be implemented by each server backend. */

void LCISD_server_init(LCIS_server_t* s);
void LCISD_server_fina(LCIS_server_t s);
static inline LCIS_mr_t LCISD_rma_reg(LCIS_endpoint_t endpoint_pp, void* buf,
                                      size_t size);
static inline void LCISD_rma_dereg(LCIS_mr_t mr);
static inline LCIS_rkey_t LCISD_rma_rkey(LCIS_mr_t mr);

void LCISD_endpoint_init(LCIS_server_t server_pp, LCIS_endpoint_t* endpoint_pp,
                         bool single_threaded);
void LCISD_endpoint_fina(LCIS_endpoint_t endpoint_pp);
static inline int LCISD_poll_cq(LCIS_endpoint_t endpoint_pp,
                                LCIS_cq_entry_t* entry);
static inline LCI_error_t LCISD_post_sends(LCIS_endpoint_t endpoint_pp,
                                           int rank, void* buf, size_t size,
                                           LCIS_meta_t meta);
static inline LCI_error_t LCISD_post_send(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size, LCIS_mr_t mr,
                                          LCIS_meta_t meta, void* ctx);
static inline LCI_error_t LCISD_post_puts(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size,
                                          uintptr_t base, LCIS_offset_t offset,
                                          LCIS_rkey_t rkey);
static inline LCI_error_t LCISD_post_put(LCIS_endpoint_t endpoint_pp, int rank,
                                         void* buf, size_t size, LCIS_mr_t mr,
                                         uintptr_t base, LCIS_offset_t offset,
                                         LCIS_rkey_t rkey, void* ctx);
static inline LCI_error_t LCISD_post_putImms(LCIS_endpoint_t endpoint_pp,
                                             int rank, void* buf, size_t size,
                                             uintptr_t base,
                                             LCIS_offset_t offset,
                                             LCIS_rkey_t rkey, uint32_t meta);
static inline LCI_error_t LCISD_post_putImm(LCIS_endpoint_t endpoint_pp,
                                            int rank, void* buf, size_t size,
                                            LCIS_mr_t mr, uintptr_t base,
                                            LCIS_offset_t offset,
                                            LCIS_rkey_t rkey, LCIS_meta_t meta,
                                            void* ctx);
static inline LCI_error_t LCISD_post_recv(LCIS_endpoint_t endpoint_pp,
                                          void* buf, uint32_t size,
                                          LCIS_mr_t mr, void* ctx);

#ifdef LCI_USE_SERVER_OFI
#include "backend/ofi/server_ofi.h"
#endif
#ifdef LCI_USE_SERVER_IBV
#include "backend/ibv/server_ibv.h"
#include "backend/ibv/lcisi_ibv_detail.h"
#endif
#ifdef LCI_USE_SERVER_UCX
#include "backend/ucx/server_ucx.h"
#include "backend/ucx/lcisi_ucx_detail.h"
#endif

#define LCIS_endpoint_super(endpoint) (((LCISI_endpoint_t*)endpoint)->super)

#define LCISI_CS_ENTER(endpoint, ret)                                  \
  if (LCI_BACKEND_TRY_LOCK_MODE == LCI_BACKEND_TRY_LOCK_GLOBAL &&      \
      !LCIU_try_acquire_spinlock(&LCIS_endpoint_super(endpoint).lock)) \
    return ret;                                                        \
  else if (LCI_BACKEND_TRY_LOCK_MODE == LCI_BACKEND_LOCK_GLOBAL)       \
    LCIU_acquire_spinlock(&LCIS_endpoint_super(endpoint).lock);

#define LCISI_CS_EXIT(endpoint)                                   \
  if (LCI_BACKEND_TRY_LOCK_MODE == LCI_BACKEND_TRY_LOCK_GLOBAL || \
      LCI_BACKEND_TRY_LOCK_MODE == LCI_BACKEND_LOCK_GLOBAL)       \
    LCIU_release_spinlock(&LCIS_endpoint_super(endpoint).lock);

/* Wrapper functions */
static inline void LCIS_server_init(LCIS_server_t* s) { LCISD_server_init(s); }

static inline void LCIS_server_fina(LCIS_server_t s) { LCISD_server_fina(s); }

static inline LCIS_rkey_t LCIS_rma_rkey(LCIS_mr_t mr)
{
  return LCISD_rma_rkey(mr);
}

static inline LCIS_mr_t LCIS_rma_reg(LCIS_endpoint_t endpoint_pp, void* buf,
                                     size_t size)
{
  LCII_PCOUNTER_START(net_mem_reg_timer);
  LCIS_mr_t mr = LCISD_rma_reg(endpoint_pp, buf, size);
  LCII_PCOUNTER_END(net_mem_reg_timer);
  LCI_DBG_Log(LCI_LOG_TRACE, "server-reg",
              "LCIS_rma_reg: mr %p buf %p size %lu rkey %lu\n", mr.mr_p, buf,
              size, LCISD_rma_rkey(mr));
  return mr;
}

static inline void LCIS_rma_dereg(LCIS_mr_t mr)
{
  LCI_DBG_Log(LCI_LOG_TRACE, "server-reg",
              "LCIS_rma_dereg: mr %p buf %p size %lu rkey %lu\n", mr.mr_p,
              mr.address, mr.length, LCISD_rma_rkey(mr));
  LCII_PCOUNTER_START(net_mem_dereg_timer);
  LCISD_rma_dereg(mr);
  LCII_PCOUNTER_END(net_mem_dereg_timer);
}

static inline void LCIS_endpoint_init(LCIS_server_t server_pp,
                                      LCIS_endpoint_t* endpoint_pp,
                                      bool single_threaded)
{
  LCISD_endpoint_init(server_pp, endpoint_pp, single_threaded);
  LCIU_spinlock_init(&LCIS_endpoint_super(*endpoint_pp).lock);
}

static inline void LCIS_endpoint_fina(LCIS_endpoint_t endpoint_pp)
{
  LCIU_spinlock_fina(&LCIS_endpoint_super(endpoint_pp).lock);
  LCISD_endpoint_fina(endpoint_pp);
}

static inline int LCIS_poll_cq(LCIS_endpoint_t endpoint_pp,
                               LCIS_cq_entry_t* entry)
{
  LCII_PCOUNTER_ADD(net_poll_cq_attempts, 1);
  LCISI_CS_ENTER(endpoint_pp, 0);
  int ret = LCISD_poll_cq(endpoint_pp, entry);
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}

static inline LCI_error_t LCIS_post_sends(LCIS_endpoint_t endpoint_pp, int rank,
                                          void* buf, size_t size,
                                          LCIS_meta_t meta)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_sends: rank %d buf %p size %lu meta %d\n", rank, buf,
              size, meta);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret = LCISD_post_sends(endpoint_pp, rank, buf, size, meta);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_sends_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_send(LCIS_endpoint_t endpoint_pp, int rank,
                                         void* buf, size_t size, LCIS_mr_t mr,
                                         LCIS_meta_t meta, void* ctx)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_send: rank %d buf %p size %lu mr %p meta %d ctx %p\n",
              rank, buf, size, mr.mr_p, meta, ctx);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret =
      LCISD_post_send(endpoint_pp, rank, buf, size, mr, meta, ctx);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_send_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_puts(LCIS_endpoint_t endpoint_pp, int rank,
                                         void* buf, size_t size, uintptr_t base,
                                         LCIS_offset_t offset, LCIS_rkey_t rkey)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_puts: rank %d buf %p size %lu base %p offset %lu "
              "rkey %lu\n",
              rank, buf, size, (void*)base, offset, rkey);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret =
      LCISD_post_puts(endpoint_pp, rank, buf, size, base, offset, rkey);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_sends_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_put(LCIS_endpoint_t endpoint_pp, int rank,
                                        void* buf, size_t size, LCIS_mr_t mr,
                                        uintptr_t base, LCIS_offset_t offset,
                                        LCIS_rkey_t rkey, void* ctx)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_put: rank %d buf %p size %lu mr %p base %p "
              "offset %lu rkey %lu ctx %p\n",
              rank, buf, size, mr.mr_p, (void*)base, offset, rkey, ctx);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret =
      LCISD_post_put(endpoint_pp, rank, buf, size, mr, base, offset, rkey, ctx);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_send_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_putImms(LCIS_endpoint_t endpoint_pp,
                                            int rank, void* buf, size_t size,
                                            uintptr_t base,
                                            LCIS_offset_t offset,
                                            LCIS_rkey_t rkey, uint32_t meta)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_putImms: rank %d buf %p size %lu base %p offset %lu "
              "rkey %lu meta %d\n",
              rank, buf, size, (void*)base, offset, rkey, meta);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret = LCISD_post_putImms(endpoint_pp, rank, buf, size, base,
                                       offset, rkey, meta);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_send_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_putImm(LCIS_endpoint_t endpoint_pp,
                                           int rank, void* buf, size_t size,
                                           LCIS_mr_t mr, uintptr_t base,
                                           LCIS_offset_t offset,
                                           LCIS_rkey_t rkey, LCIS_meta_t meta,
                                           void* ctx)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_putImm: rank %d buf %p size %lu mr %p base %p "
              "offset %lu rkey %lu meta %u ctx %p\n",
              rank, buf, size, mr.mr_p, (void*)base, offset, rkey, meta, ctx);
#ifdef LCI_ENABLE_SLOWDOWN
  LCIU_spin_for_nsec(LCI_SEND_SLOW_DOWN_USEC * 1000);
#endif
  LCI_error_t ret = LCISD_post_putImm(endpoint_pp, rank, buf, size, mr, base,
                                      offset, rkey, meta, ctx);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_send_posted, (int64_t)size);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_send_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  } else if (ret == LCI_ERR_RETRY_NOMEM) {
    LCII_PCOUNTER_ADD(net_send_failed_nomem, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}
static inline LCI_error_t LCIS_post_recv(LCIS_endpoint_t endpoint_pp, void* buf,
                                         uint32_t size, LCIS_mr_t mr, void* ctx)
{
  LCISI_CS_ENTER(endpoint_pp, LCI_ERR_RETRY);
  LCI_DBG_Log(LCI_LOG_TRACE, "server",
              "LCIS_post_recv: buf %p size %u mr %p user_context %p\n", buf,
              size, mr.mr_p, ctx);
  LCI_error_t ret = LCISD_post_recv(endpoint_pp, buf, size, mr, ctx);
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(net_recv_posted, 1);
  } else if (ret == LCI_ERR_RETRY_LOCK) {
    LCII_PCOUNTER_ADD(net_recv_failed_lock, 1);
    ret = LCI_ERR_RETRY;
  }
  LCISI_CS_EXIT(endpoint_pp);
  return ret;
}

#endif
