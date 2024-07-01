#include "runtime/lcii.h"

static inline LCI_error_t LCII_progress_bq(LCI_device_t device)
{
  if (LCII_bq_is_empty(&device->bq)) return LCI_ERR_RETRY;
  if (!LCIU_try_acquire_spinlock(&device->bq_spinlock)) return LCI_ERR_RETRY;
  LCI_error_t ret = LCI_ERR_RETRY;
  LCII_bq_entry_t* entry = LCII_bq_top(&device->bq);
  if (entry != NULL) {
    if (entry->bqe_type == LCII_BQ_SENDS) {
      ret = LCIS_post_sends(device->endpoint_progress->endpoint, entry->rank,
                            entry->buf, entry->size, entry->meta);
    } else if (entry->bqe_type == LCII_BQ_SEND) {
      ret = LCIS_post_send(device->endpoint_progress->endpoint, entry->rank,
                           entry->buf, entry->size, entry->mr, entry->meta,
                           entry->ctx);
    } else if (entry->bqe_type == LCII_BQ_PUT) {
      ret = LCIS_post_put(device->endpoint_progress->endpoint, entry->rank,
                          entry->buf, entry->size, entry->mr, entry->base,
                          entry->offset, entry->rkey, entry->ctx);
    } else if (entry->bqe_type == LCII_BQ_PUTIMM) {
      ret =
          LCIS_post_putImm(device->endpoint_progress->endpoint, entry->rank,
                           entry->buf, entry->size, entry->mr, entry->base,
                           entry->offset, entry->rkey, entry->meta, entry->ctx);
    } else {
      LCI_DBG_Assert(false, "Unknown bqe_type (%d)!\n", entry->bqe_type);
    }
    if (ret == LCI_OK) {
      LCI_DBG_Log(LCI_LOG_TRACE, "bq", "Pop from backlog queue: type %d\n",
                  entry->bqe_type);
      LCII_bq_pop(&device->bq);
      if (entry->bqe_type == LCII_BQ_SENDS) LCIU_free(entry->buf);
    }
  }
  LCIU_release_spinlock(&device->bq_spinlock);
  return ret;
}

LCI_error_t LCII_poll_cq(LCII_endpoint_t* endpoint)
{
  int ret = LCI_ERR_RETRY;
  // poll progress endpoint completion queue
  LCIS_cq_entry_t entry[LCI_CQ_MAX_POLL];
  int count = LCIS_poll_cq(endpoint->endpoint, entry);
  if (count > 0) {
    ret = LCI_OK;
    LCII_PCOUNTER_START(useful_progress_timer);
  } else {
    LCI_DBG_Assert(count >= 0, "ibv_poll_cq returns error %d\n", count);
  }
  for (int i = 0; i < count; i++) {
#ifdef LCI_ENABLE_SLOWDOWN
    LCIU_spin_for_nsec(LCI_RECV_SLOW_DOWN_USEC * 1000);
#endif
    if (entry[i].opcode == LCII_OP_RECV) {
      // two-sided recv.
      LCI_DBG_Log(LCI_LOG_TRACE, "device",
                  "complete recv: packet %p rank %d length %lu imm_data %u\n",
                  entry[i].ctx, entry[i].rank, entry[i].length,
                  entry[i].imm_data);
      LCIS_serve_recv((LCII_packet_t*)entry[i].ctx, entry[i].rank,
                      entry[i].length, entry[i].imm_data);
      LCII_PCOUNTER_START(update_posted_recv);
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
      atomic_fetch_sub_explicit(&endpoint->recv_posted, 1,
                                LCIU_memory_order_relaxed);
#else
      --endpoint->recv_posted;
#endif
      LCII_PCOUNTER_END(update_posted_recv);
    } else if (entry[i].opcode == LCII_OP_RDMA_WRITE) {
      LCI_DBG_Log(LCI_LOG_TRACE, "device", "complete write: imm_data %u\n",
                  entry[i].imm_data);
      if (entry[i].ctx != NULL) {
        LCII_free_packet((LCII_packet_t*)entry[i].ctx);
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
        atomic_fetch_sub_explicit(&endpoint->recv_posted, 1,
                                  LCIU_memory_order_relaxed);
#else
        --endpoint->recv_posted;
#endif
      }
      LCIS_serve_rdma(entry[i].imm_data);
    } else {
      // entry[i].opcode == LCII_OP_SEND
      LCI_DBG_Log(LCI_LOG_TRACE, "device", "complete send: address %p\n",
                  (void*)entry[i].ctx);
      LCII_PCOUNTER_ADD(net_send_comp, 1);
      if (entry[i].ctx == NULL) {
        continue;
      }
      LCIS_serve_send((void*)entry[i].ctx);
    }
  }
  if (ret == LCI_OK) {
    LCII_PCOUNTER_END(useful_progress_timer);
  }
  return ret;
}

LCI_error_t LCII_fill_rq(LCII_endpoint_t* endpoint, bool block)
{
  int ret = LCI_ERR_RETRY;
  LCII_PCOUNTER_START(refill_rq_timer);
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
  while (atomic_load_explicit(&endpoint->recv_posted, memory_order_relaxed) <
         LCI_SERVER_MAX_RECVS) {
#else
  while (endpoint->recv_posted < LCI_SERVER_MAX_RECVS) {
#endif
    bool post_recv_succeed = false;

    // First, get a packet.
    LCII_PCOUNTER_START(get_recv_packet_timer);
    LCII_packet_t* packet = LCII_alloc_packet_nb(endpoint->device->heap->pool);
    LCII_PCOUNTER_END(get_recv_packet_timer);
    if (packet == NULL) {
      LCII_PCOUNTER_ADD(net_recv_failed_nopacket, 1);
    } else {
      // We got the packet, post a recv
      LCII_PCOUNTER_START(post_recv_timer);
      // TODO: figure out what is the right poolid to set
      // packet->context.poolid = lc_pool_get_local(endpoint->device->pkpool);
      LCI_error_t rc = LCIS_post_recv(
          endpoint->endpoint, packet->data.address, LCI_MEDIUM_SIZE,
          endpoint->device->heap_segment->mr, packet);
      if (rc == LCI_OK) {
        LCII_PCOUNTER_START(update_posted_recv);
#ifdef LCI_ENABLE_MULTITHREAD_PROGRESS
        atomic_fetch_add_explicit(&endpoint->recv_posted, 1,
                                  LCIU_memory_order_relaxed);
#else
        ++endpoint->recv_posted;
#endif
        LCII_PCOUNTER_END(update_posted_recv);
        post_recv_succeed = true;
        ret = LCI_OK;
      } else {
        LCII_free_packet(packet);
      }
      LCII_PCOUNTER_END(post_recv_timer);
    }
    if (post_recv_succeed || block) {
      continue;
    } else {
      break;
    }
  }
  LCII_PCOUNTER_END(refill_rq_timer);
  return ret;
}

LCI_error_t LCI_progress(LCI_device_t device)
{
  int ret = LCI_ERR_RETRY;
  // we want to make progress on the endpoint_progress as much as possible
  // to speed up rendezvous protocol
  while (LCI_ENABLE_PRG_NET_ENDPOINT &&
         LCII_poll_cq(device->endpoint_progress) == LCI_OK) {
    ret = LCI_OK;
  }
  while (LCII_progress_bq(device) == LCI_OK) {
    ret = LCI_OK;
  }
  // Make sure we always have enough packet, but do not block.
  if (LCI_ENABLE_PRG_NET_ENDPOINT &&
      LCII_fill_rq(device->endpoint_progress, false) == LCI_OK) {
    ret = LCI_OK;
  }
  if (LCII_poll_cq(device->endpoint_worker) == LCI_OK) {
    ret = LCI_OK;
  }
  // Make sure we always have enough packet, but do not block.
  if (LCII_fill_rq(device->endpoint_worker, false) == LCI_OK) {
    ret = LCI_OK;
  }
  LCII_PCOUNTER_ADD(progress_call, 1);
  return ret;
}
