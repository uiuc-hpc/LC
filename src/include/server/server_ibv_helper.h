#include "config.h"

#define ALIGNMENT (lcg_page_size)
#define MAX_CQ 16

#define IBV_IMM_RTR ((uint32_t) 1<<31)

#ifdef LC_SERVER_DEBUG
#define IBV_SAFECALL(x)                                               \
  {                                                                   \
    int err = (x);                                                    \
    if (err) {                                                        \
      fprintf(stderr, "err : %d (%s:%d)\n", err, __FILE__, __LINE__); \
      exit(EXIT_FAILURE);                                             \
    }                                                                 \
  }                                                                   \
  while (0)                                                           \
    ;
#else
#define IBV_SAFECALL(x) \
  {                     \
    x;                  \
  }                     \
  while (0)             \
    ;
#endif

#define MIN(x, y) ((x) < (y) ? (x) : (y))

struct conn_ctx {
  uint64_t addr;
  uint32_t rkey;
  uint32_t qp_num;
  uint16_t lid;
  union ibv_gid gid;
};

extern int lcg_ndev;

static inline struct ibv_mr* ibv_mem_malloc(lc_server* s, size_t size)
{
  int mr_flags =
      IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE;
  void* ptr = 0;
  posix_memalign(&ptr, lcg_page_size, size + lcg_page_size);
  return ibv_reg_mr(s->dev_pd, ptr, size, mr_flags);
}

static inline struct ibv_qp* qp_create(lc_server* s)
{
  struct ibv_qp_init_attr qp_init_attr;
  qp_init_attr.qp_context = 0;
  qp_init_attr.send_cq = s->send_cq;
  qp_init_attr.recv_cq = s->recv_cq;
  qp_init_attr.srq = s->dev_srq;
  qp_init_attr.cap.max_send_wr = 256; //(uint32_t)dev_attr->max_qp_wr;
  qp_init_attr.cap.max_recv_wr = 1; //(uint32_t)dev_attr->max_qp_wr;
  // -- this affect the size of (TODO:tune later).
  qp_init_attr.cap.max_send_sge = 16; // this allows 128 inline.
  qp_init_attr.cap.max_recv_sge = 1;
  qp_init_attr.cap.max_inline_data = 0;
  qp_init_attr.qp_type = IBV_QPT_RC;
  qp_init_attr.sq_sig_all = 0;
  struct ibv_qp* qp = ibv_create_qp(s->dev_pd, &qp_init_attr);
  if (!qp) {
    fprintf(stderr, "Unable to create queue pair\n");
    exit(EXIT_FAILURE);
  }
  s->max_inline = MIN(qp_init_attr.cap.max_inline_data, LC_MAX_INLINE);
  return qp;
}

static inline void qp_init(struct ibv_qp* qp, int port)
{
  struct ibv_qp_attr attr;
  memset(&attr, 0, sizeof(struct ibv_qp_attr));
  attr.qp_state = IBV_QPS_INIT;
  attr.port_num = port;
  attr.pkey_index = 0;
  attr.qp_access_flags =
      IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE;
  int flags =
      IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT | IBV_QP_ACCESS_FLAGS;
  int rc = ibv_modify_qp(qp, &attr, flags);
  if (rc != 0) {
    fprintf(stderr, "Unable to init qp\n");
    exit(EXIT_FAILURE);
  }
}

static inline void qp_to_rtr(struct ibv_qp* qp, int dev_port,
                         struct ibv_port_attr* port_attr,
                         struct conn_ctx* rctx_)
{
  struct ibv_qp_attr attr;
  memset(&attr, 0, sizeof(struct ibv_qp_attr));

  attr.qp_state = IBV_QPS_RTR;
  attr.path_mtu = port_attr->active_mtu;
  attr.dest_qp_num = rctx_->qp_num;
  attr.rq_psn = 0;
  attr.max_dest_rd_atomic = 1;
  attr.min_rnr_timer = 12;

  attr.ah_attr.is_global = 0;
  attr.ah_attr.dlid = rctx_->lid;
  attr.ah_attr.sl = 0;
  attr.ah_attr.src_path_bits = 0;
  attr.ah_attr.port_num = dev_port;

  memcpy(&attr.ah_attr.grh.dgid, &rctx_->gid, 16);
  attr.ah_attr.grh.sgid_index = 0;  // gid

  int flags = IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN |
              IBV_QP_RQ_PSN | IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER;

  int rc = ibv_modify_qp(qp, &attr, flags);
  if (rc != 0) {
    fprintf(stderr, "failed to modify QP state to RTR\n");
    exit(EXIT_FAILURE);
  }
}

static inline void qp_to_rts(struct ibv_qp* qp)
{
  struct ibv_qp_attr attr;
  memset(&attr, 0, sizeof(struct ibv_qp_attr));

  attr.qp_state = IBV_QPS_RTS;
  attr.timeout = 0x12;
  attr.retry_cnt = 7;
  attr.rnr_retry = 7;
  attr.sq_psn = 0;
  attr.max_rd_atomic = 1;

  int flags = IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT |
              IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN | IBV_QP_MAX_QP_RD_ATOMIC;
  int rc = ibv_modify_qp(qp, &attr, flags);
  if (rc != 0) {
    fprintf(stderr, "failed to modify QP state to RTS\n");
    exit(EXIT_FAILURE);
  }
}

static inline uintptr_t _real_server_reg(lc_server* s, void* buf, size_t size)
{
  int mr_flags =
      IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE;
  return (uintptr_t)ibv_reg_mr(s->dev_pd, buf, size, mr_flags);
}

static inline void _real_server_dereg(uintptr_t mem)
{
  ibv_dereg_mr((struct ibv_mr*)mem);
}

static inline void ibv_post_recv_(lc_server* s, lc_packet* p)
{
  if (p == NULL) {
    if (s->recv_posted < LC_SERVER_MAX_RCVS / 2 && !lcg_deadlock) {
      lcg_deadlock = 1;
      #ifdef LC_SERVER_DEBUG
      printf("WARNING-LC: deadlock alert\n");
      #endif
    }
    return;
  }

  struct ibv_sge sg = {
      .addr = (uintptr_t)(&p->data),
      .length = POST_MSG_SIZE,
      .lkey = s->heap->lkey,
  };

  struct ibv_recv_wr wr = {
      .wr_id = (uintptr_t) & (p->context),
      .next = 0,
      .sg_list = &sg,
      .num_sge = 1,
  };

  p->context.poolid = lc_pool_get_local(s->pkpool);

  struct ibv_recv_wr* bad_wr = 0;
  IBV_SAFECALL(ibv_post_srq_recv(s->dev_srq, &wr, &bad_wr));

  if (++s->recv_posted == LC_SERVER_MAX_RCVS && lcg_deadlock)
    lcg_deadlock = 0;
}
