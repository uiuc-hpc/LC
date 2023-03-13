#ifndef SERVER_H_
#define SERVER_H_

#include "config.h"

struct lc_server;
typedef struct lc_server lc_server;

extern volatile uint32_t lc_next_rdma_key;

#define SERVER_COMMON \
  int id; \
  lc_pool* pkpool; \
  uintptr_t curr_addr; \
  struct lc_rep* rep; \
  size_t recv_posted; \
  uintptr_t heap_addr; \

static inline void lc_serve_recv(lc_packet* p, lc_proto proto);
static inline void lc_serve_imm(lc_packet* p);
static inline void lc_serve_recv_rdma(lc_packet*, lc_proto proto);
static inline void lc_serve_send(lc_packet* p);

// Following functions are required to be implemented by each server backend.

static inline int lc_server_progress(lc_server* s);
static inline void lc_server_init(int id, lc_server** dev);
static inline void lc_server_finalize(lc_server* s);

static inline uintptr_t lc_server_rma_reg(lc_server* s, void* buf, size_t size);
static inline void lc_server_rma_dereg(uintptr_t mem);
static inline uint32_t lc_server_rma_key(uintptr_t mem);

static inline void lc_server_sends(lc_server* s, void* rep, void* ubuf,
                                   size_t size, uint32_t proto);
static inline void lc_server_sendm(lc_server* s, void* rep, size_t size,
                                   lc_packet* p, uint32_t proto);
static inline void lc_server_puts(lc_server* s, void* rep, void* buf,
                                   uintptr_t base, uint32_t offset,
                                   uint32_t rkey, uint32_t meta, size_t size);
static inline void lc_server_putm(lc_server* s, void* rep, uintptr_t base,
                                  uint32_t offset, uint32_t rkey, size_t size,
                                  uint32_t meta, lc_packet* p);
static inline void lc_server_putl(lc_server* s, void* rep, void* buf,
                                  uintptr_t base, uint32_t offset,
                                  uint32_t rkey, size_t size, uint32_t meta,
                                  lc_packet* p);

static inline void lc_server_rma_rtr(lc_server* s, void* rep, void* buf,
                                     uintptr_t addr, uint32_t rkey, size_t size,
                                     uint32_t sid, lc_packet* p);

static inline void* lc_server_heap_ptr(lc_server* s);

#ifdef LCI_USE_SERVER_OFI
#include "server_ofi.h"
#endif

#ifdef LCI_USE_SERVER_PSM
#include "server_psm2.h"
#endif

#ifdef LCI_USE_SERVER_IBV
#include "server_ibv.h"
#endif

#endif
