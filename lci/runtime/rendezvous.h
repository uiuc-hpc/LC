#ifndef LCI_RENDEZVOUS_H
#define LCI_RENDEZVOUS_H

// wrapper for send and put
static inline void LCIS_post_sends_bq(LCII_backlog_queue_t* bq_p,
                                      LCIU_spinlock_t* bq_spinlock_p,
                                      LCIS_endpoint_t endpoint, int rank,
                                      void* buf, size_t size, LCIS_meta_t meta)
{
  if (LCII_bq_is_empty(bq_p)) {
    LCI_error_t ret = LCIS_post_sends(endpoint, rank, buf, size, meta);
    if (ret == LCI_OK)
      return;
    else {
      LCI_Assert(ret == LCI_ERR_RETRY, "fatal error!\n");
    }
  }
  // push to backlog queue
  LCI_DBG_Log(LCI_LOG_TRACE, "bq",
              "Pushed to backlog queue (sends): "
              "post sends: rank %d buf %p size %lu meta %d\n",
              rank, buf, size, meta);
  LCII_bq_entry_t* entry = LCIU_malloc(sizeof(struct LCII_bq_entry_t));
  entry->bqe_type = LCII_BQ_SENDS;
  entry->rank = rank;
  entry->buf = LCIU_malloc(size);
  memcpy(entry->buf, buf, size);
  entry->size = size;
  entry->meta = meta;
  LCIU_acquire_spinlock(bq_spinlock_p);
  LCII_bq_push(bq_p, entry);
  LCIU_release_spinlock(bq_spinlock_p);
}

static inline void LCIS_post_send_bq(LCII_backlog_queue_t* bq_p,
                                     LCIU_spinlock_t* bq_spinlock_p,
                                     LCIS_endpoint_t endpoint, int rank,
                                     void* buf, size_t size, LCIS_mr_t mr,
                                     LCIS_meta_t meta, void* ctx)
{
  if (LCII_bq_is_empty(bq_p)) {
    LCI_error_t ret = LCIS_post_send(endpoint, rank, buf, size, mr, meta, ctx);
    if (ret == LCI_OK)
      return;
    else {
      LCI_Assert(ret == LCI_ERR_RETRY, "fatal error!\n");
    }
  }
  // push to backlog queue
  LCI_DBG_Log(LCI_LOG_TRACE, "bq",
              "Pushed to backlog queue (send): "
              "rank %d buf %p size %lu mr %p meta %d ctx %p\n",
              rank, buf, size, mr.mr_p, meta, ctx);
  LCII_bq_entry_t* entry = LCIU_malloc(sizeof(struct LCII_bq_entry_t));
  entry->bqe_type = LCII_BQ_SEND;
  entry->rank = rank;
  entry->buf = buf;
  entry->size = size;
  entry->mr = mr;
  entry->meta = meta;
  entry->ctx = ctx;
  LCIU_acquire_spinlock(bq_spinlock_p);
  LCII_bq_push(bq_p, entry);
  LCIU_release_spinlock(bq_spinlock_p);
}

static inline void LCIS_post_put_bq(LCII_backlog_queue_t* bq_p,
                                    LCIU_spinlock_t* bq_spinlock_p,
                                    LCIS_endpoint_t endpoint, int rank,
                                    void* buf, size_t size, LCIS_mr_t mr,
                                    uintptr_t base, LCIS_offset_t offset,
                                    LCIS_rkey_t rkey, void* ctx)
{
  if (LCII_bq_is_empty(bq_p)) {
    LCI_error_t ret =
        LCIS_post_put(endpoint, rank, buf, size, mr, base, offset, rkey, ctx);
    if (ret == LCI_OK)
      return;
    else {
      LCI_Assert(ret == LCI_ERR_RETRY, "fatal error!\n");
    }
  }
  // push to backlog queue
  LCI_DBG_Log(LCI_LOG_TRACE, "bq",
              "Pushed to backlog queue (put): "
              "rank %d buf %p size %lu mr %p base %p "
              "offset %lu rkey %lu ctx %p\n",
              rank, buf, size, mr.mr_p, (void*)base, offset, rkey, ctx);
  LCII_bq_entry_t* entry = LCIU_malloc(sizeof(struct LCII_bq_entry_t));
  entry->bqe_type = LCII_BQ_PUT;
  entry->rank = rank;
  entry->buf = buf;
  entry->size = size;
  entry->mr = mr;
  entry->ctx = ctx;
  entry->base = base;
  entry->offset = offset;
  entry->rkey = rkey;
  LCIU_acquire_spinlock(bq_spinlock_p);
  LCII_bq_push(bq_p, entry);
  LCIU_release_spinlock(bq_spinlock_p);
}

static inline void LCIS_post_putImm_bq(LCII_backlog_queue_t* bq_p,
                                       LCIU_spinlock_t* bq_spinlock_p,
                                       LCIS_endpoint_t endpoint, int rank,
                                       void* buf, size_t size, LCIS_mr_t mr,
                                       uintptr_t base, LCIS_offset_t offset,
                                       LCIS_rkey_t rkey, LCIS_meta_t meta,
                                       void* ctx)
{
  if (LCII_bq_is_empty(bq_p)) {
    LCI_error_t ret = LCIS_post_putImm(endpoint, rank, buf, size, mr, base,
                                       offset, rkey, meta, ctx);
    if (ret == LCI_OK)
      return;
    else {
      LCI_Assert(ret == LCI_ERR_RETRY, "fatal error!\n");
    }
  }
  // push to backlog queue
  LCI_DBG_Log(LCI_LOG_TRACE, "bq",
              "Pushed to backlog queue (putImm): "
              "rank %d buf %p size %lu mr %p base %p "
              "offset %lu rkey %lu meta %u ctx %p\n",
              rank, buf, size, mr.mr_p, (void*)base, offset, rkey, meta, ctx);
  LCII_bq_entry_t* entry = LCIU_malloc(sizeof(struct LCII_bq_entry_t));
  entry->bqe_type = LCII_BQ_PUTIMM;
  entry->rank = rank;
  entry->buf = buf;
  entry->size = size;
  entry->mr = mr;
  entry->meta = meta;
  entry->ctx = ctx;
  entry->base = base;
  entry->offset = offset;
  entry->rkey = rkey;
  LCIU_acquire_spinlock(bq_spinlock_p);
  LCII_bq_push(bq_p, entry);
  LCIU_release_spinlock(bq_spinlock_p);
}

static void LCII_env_init_rdv_protocol()
{
  LCT_dict_str_int_t dict[] = {
      {"write", LCI_RDV_WRITE},
      {"writeimm", LCI_RDV_WRITEIMM},
  };
  char* p = getenv("LCI_RDV_PROTOCOL");
  if (!p) p = LCI_RDV_PROTOCOL_DEFAULT;
  bool succeed = LCT_str_int_search(dict, sizeof(dict) / sizeof(dict[0]), p,
                                    LCI_RDV_WRITE, (int*)&LCI_RDV_PROTOCOL);
  if (!succeed) {
    LCI_Warn("Unknown LCI_RDV_PROTOCOL %s. Use the default type: write\n",
             getenv("LCI_RDV_PROTOCOL"));
  }
  LCI_Log(LCI_LOG_INFO, "rdv", "Set LCI_RDV_PROTOCOL to %d\n",
          LCI_RDV_PROTOCOL);
}

static inline void LCII_rts_fill_rbuffer_info(
    struct LCII_packet_rtr_rbuffer_info_t* p, LCI_lbuffer_t lbuffer)
{
  p->remote_addr_base = (uintptr_t)lbuffer.segment->mr.address;
  p->remote_addr_offset = (uintptr_t)lbuffer.address - p->remote_addr_base;
  p->rkey = LCIS_rma_rkey(lbuffer.segment->mr);
}

static inline void LCII_handle_rts(LCI_endpoint_t ep, LCII_packet_t* packet,
                                   int src_rank, uint16_t tag,
                                   LCII_context_t* rdv_ctx, bool is_in_progress)
{
  // Extract information from the received RTS packet
  LCII_rdv_type_t rdv_type = packet->data.rts.rdv_type;
  LCI_DBG_Log(LCI_LOG_TRACE, "rdv", "handle rts: rdv_type %d\n", rdv_type);
  if (!rdv_ctx) {
    LCI_DBG_Assert(rdv_type == LCII_RDV_1SIDED || rdv_type == LCII_RDV_IOVEC,
                   "");
    rdv_ctx = LCIU_malloc(sizeof(LCII_context_t));
  }
  if (rdv_type == LCII_RDV_2SIDED) {
    LCI_DBG_Assert(rdv_ctx->data.lbuffer.address == NULL ||
                       rdv_ctx->data.lbuffer.length >= packet->data.rts.size,
                   "the message sent by sendl (%lu) is larger than the buffer "
                   "posted by recvl (%lu)!\n",
                   packet->data.rts.size, rdv_ctx->data.lbuffer.length);
    LCI_DBG_Assert(packet->context.src_rank == src_rank, "");
    LCI_DBG_Assert(rdv_ctx->tag == tag, "");
  }
  rdv_ctx->data.lbuffer.length = packet->data.rts.size;
  rdv_ctx->rank = src_rank;
  if (rdv_type == LCII_RDV_1SIDED || rdv_type == LCII_RDV_IOVEC) {
    // For 2sided, we already set these fields when posting the recvl.
    rdv_ctx->tag = tag;
    rdv_ctx->user_context = NULL;
    rdv_ctx->completion = ep->default_comp;
    LCII_initilize_comp_attr(rdv_ctx->comp_attr);
    LCII_comp_attr_set_rdv_type(rdv_ctx->comp_attr, packet->data.rts.rdv_type);
    LCII_comp_attr_set_comp_type(rdv_ctx->comp_attr, ep->msg_comp_type);
  }

  // Prepare the data
  LCII_PCOUNTER_START(rts_mem_timer);
  if (rdv_type == LCII_RDV_2SIDED) {
    if (rdv_ctx->data.lbuffer.address == NULL) {
      LCI_lbuffer_alloc(ep->device, packet->data.rts.size,
                        &rdv_ctx->data.lbuffer);
    }
    if (rdv_ctx->data.lbuffer.segment == LCI_SEGMENT_ALL) {
      LCI_DBG_Assert(LCII_comp_attr_get_dereg(rdv_ctx->comp_attr) == 1, "\n");
      LCI_memory_register(ep->device, rdv_ctx->data.lbuffer.address,
                          rdv_ctx->data.lbuffer.length,
                          &rdv_ctx->data.lbuffer.segment);
    } else {
      LCI_DBG_Assert(LCII_comp_attr_get_dereg(rdv_ctx->comp_attr) == 0, "\n");
    }
  } else if (rdv_type == LCII_RDV_1SIDED) {
    LCI_lbuffer_alloc(ep->device, rdv_ctx->data.lbuffer.length,
                      &rdv_ctx->data.lbuffer);
    rdv_ctx->data_type = LCI_LONG;
  } else {
    rdv_ctx->data.iovec.count = packet->data.rts.count;
    rdv_ctx->data.iovec.piggy_back.length = packet->data.rts.piggy_back_size;
    rdv_ctx->data.iovec.piggy_back.address =
        LCIU_malloc(packet->data.rts.piggy_back_size);
    memcpy(rdv_ctx->data.iovec.piggy_back.address,
           (void*)&packet->data.rts.size_p[packet->data.rts.count],
           packet->data.rts.piggy_back_size);
    rdv_ctx->data.iovec.lbuffers =
        LCIU_malloc(sizeof(LCI_lbuffer_t) * packet->data.rts.count);
    for (int i = 0; i < packet->data.rts.count; ++i) {
      LCI_lbuffer_alloc(ep->device, packet->data.rts.size_p[i],
                        &rdv_ctx->data.iovec.lbuffers[i]);
    }
    rdv_ctx->data_type = LCI_IOVEC;
  }
  LCII_PCOUNTER_END(rts_mem_timer);

  // Prepare the RTR context
  LCII_context_t* rtr_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rtr_ctx->data.packet = packet;
  LCII_initilize_comp_attr(rtr_ctx->comp_attr);
  LCII_comp_attr_set_free_packet(rtr_ctx->comp_attr, 1);

  // Prepare the RTR packet
  // reuse the rts packet as rtr packet
  packet->context.poolid = LCII_POOLID_LOCAL;
  if (LCI_RDV_PROTOCOL == LCI_RDV_WRITEIMM && rdv_type != LCII_RDV_IOVEC) {
    // IOVEC does not support writeimm for now
    uint64_t ctx_key;
    int result =
        LCM_archive_put(ep->ctx_archive_p, (uintptr_t)rdv_ctx, &ctx_key);
    // TODO: be able to pass back pressure to user
    LCI_Assert(result == LCM_SUCCESS, "Archive is full!\n");
    packet->data.rtr.recv_ctx_key = ctx_key;
  } else {
    packet->data.rtr.recv_ctx = (uintptr_t)rdv_ctx;
  }
  if (rdv_ctx->data_type == LCI_LONG) {
    LCII_rts_fill_rbuffer_info(&packet->data.rtr.rbuffer_info_p[0],
                               rdv_ctx->data.lbuffer);
  } else {
    LCI_DBG_Assert(rdv_ctx->data_type == LCI_IOVEC, "");
    for (int i = 0; i < rdv_ctx->data.iovec.count; ++i) {
      LCII_rts_fill_rbuffer_info(&packet->data.rtr.rbuffer_info_p[i],
                                 rdv_ctx->data.iovec.lbuffers[i]);
    }
  }

  // log
  LCI_DBG_Log(LCI_LOG_TRACE, "rdv", "send rtr: sctx %p\n",
              (void*)packet->data.rtr.send_ctx);

  // send the rtr packet
  LCIS_endpoint_t endpoint_to_use;
  if (is_in_progress) {
    endpoint_to_use = ep->device->endpoint_progress->endpoint;
  } else {
    LCI_Assert(rdv_type == LCII_RDV_2SIDED, "");
    endpoint_to_use = ep->device->endpoint_worker->endpoint;
  }
  size_t num_rbuffer_info =
      (rdv_ctx->data_type == LCI_IOVEC) ? rdv_ctx->data.iovec.count : 1;
  size_t length =
      (uintptr_t)&packet->data.rtr.rbuffer_info_p[num_rbuffer_info] -
      (uintptr_t)packet->data.address;
  LCII_PCOUNTER_START(rts_send_timer);
  LCIS_post_send_bq(ep->bq_p, ep->bq_spinlock_p, endpoint_to_use,
                    (int)rdv_ctx->rank, packet->data.address, length,
                    ep->device->heap_segment->mr,
                    LCII_MAKE_PROTO(ep->gid, LCI_MSG_RTR, 0), rtr_ctx);
  LCII_PCOUNTER_END(rts_send_timer);
}

static inline void LCII_handle_rtr(LCI_endpoint_t ep, LCII_packet_t* packet)
{
  LCII_rdv_type_t rdv_type = packet->data.rtr.rdv_type;
  LCII_context_t* ctx = (LCII_context_t*)packet->data.rtr.send_ctx;
  // Set up the "extended context" for write protocol
  void* ctx_to_pass = ctx;
  if (LCI_RDV_PROTOCOL == LCI_RDV_WRITE || rdv_type == LCII_RDV_IOVEC) {
    LCII_extended_context_t* ectx =
        LCIU_malloc(sizeof(LCII_extended_context_t));
    LCII_initilize_comp_attr(ectx->comp_attr);
    LCII_comp_attr_set_extended(ectx->comp_attr, 1);
    if (ctx->data_type == LCI_LONG) {
      atomic_init(&ectx->signal_count, 1);
    } else {
      atomic_init(&ectx->signal_count, ctx->data.iovec.count);
    }
    ectx->context = ctx;
    ectx->ep = ep;
    ectx->recv_ctx = packet->data.rtr.recv_ctx;
    ctx_to_pass = ectx;
  }
  int niters = (ctx->data_type == LCI_IOVEC) ? ctx->data.iovec.count : 1;
  for (int i = 0; i < niters; ++i) {
    LCI_lbuffer_t* lbuffer;
    if (ctx->data_type == LCI_LONG) {
      lbuffer = &ctx->data.lbuffer;
    } else {
      LCI_DBG_Assert(ctx->data_type == LCI_IOVEC, "");
      lbuffer = &ctx->data.iovec.lbuffers[i];
    }
    // register the buffer if necessary
    LCII_PCOUNTER_START(rtr_mem_reg_timer);
    if (lbuffer->segment == LCI_SEGMENT_ALL) {
      LCI_DBG_Assert(LCII_comp_attr_get_dereg(ctx->comp_attr) == 1, "\n");
      LCI_memory_register(ep->device, lbuffer->address, lbuffer->length,
                          &lbuffer->segment);
    } else {
      LCI_DBG_Assert(LCII_comp_attr_get_dereg(ctx->comp_attr) == 0, "\n");
    }
    LCII_PCOUNTER_END(rtr_mem_reg_timer);
    // issue the put/putimm
    LCII_PCOUNTER_START(rtr_put_timer);
    if (LCI_RDV_PROTOCOL == LCI_RDV_WRITE || rdv_type == LCII_RDV_IOVEC) {
      LCIS_post_put_bq(ep->bq_p, ep->bq_spinlock_p,
                       ep->device->endpoint_progress->endpoint, (int)ctx->rank,
                       lbuffer->address, lbuffer->length, lbuffer->segment->mr,
                       packet->data.rtr.rbuffer_info_p[i].remote_addr_base,
                       packet->data.rtr.rbuffer_info_p[i].remote_addr_offset,
                       packet->data.rtr.rbuffer_info_p[i].rkey, ctx_to_pass);
    } else {
      LCI_DBG_Assert(
          LCI_RDV_PROTOCOL == LCI_RDV_WRITEIMM && rdv_type != LCII_RDV_IOVEC,
          "Unexpected rdv protocol!\n");
      LCIS_post_putImm_bq(ep->bq_p, ep->bq_spinlock_p,
                          ep->device->endpoint_progress->endpoint,
                          (int)ctx->rank, lbuffer->address, lbuffer->length,
                          lbuffer->segment->mr,
                          packet->data.rtr.rbuffer_info_p[0].remote_addr_base,
                          packet->data.rtr.rbuffer_info_p[0].remote_addr_offset,
                          packet->data.rtr.rbuffer_info_p[0].rkey,
                          LCII_MAKE_PROTO(ep->gid, LCI_MSG_RDV_DATA,
                                          packet->data.rtr.recv_ctx_key),
                          ctx_to_pass);
    }
    LCII_PCOUNTER_END(rtr_put_timer);
  }
  // free the rtr packet
  LCII_free_packet(packet);
}

static inline void LCII_handle_rdv_data_local_comp(
    LCII_extended_context_t* ectx)
{
  int signal_count = atomic_fetch_sub_explicit(&ectx->signal_count, 1,
                                               LCIU_memory_order_relaxed) -
                     1;
  if (signal_count) {
    return;
  }
  LCI_DBG_Assert(signal_count == 0, "Unexpected signal!\n");
  LCII_context_t* ctx = ectx->context;
  LCI_DBG_Log(LCI_LOG_TRACE, "rdv", "send FIN: rctx %p\n",
              (void*)ectx->recv_ctx);
  LCIS_post_sends_bq(ectx->ep->bq_p, ectx->ep->bq_spinlock_p,
                     ectx->ep->device->endpoint_progress->endpoint,
                     (int)ctx->rank, &ectx->recv_ctx, sizeof(ectx->recv_ctx),
                     LCII_MAKE_PROTO(ectx->ep->gid, LCI_MSG_FIN, 0));
  LCIU_free(ectx);
  lc_ce_dispatch(ctx);
}

static inline void LCII_handle_rdv_remote_comp(LCII_context_t* ctx)
{
  // We have to count data received by remote put here.
  if (ctx->data_type == LCI_LONG) {
    LCII_PCOUNTER_ADD(net_recv_comp, ctx->data.lbuffer.length);
  } else {
    for (int i = 0; i < ctx->data.iovec.count; ++i)
      LCII_PCOUNTER_ADD(net_recv_comp, ctx->data.iovec.lbuffers[i].length);
  }
  lc_ce_dispatch(ctx);
}

static inline void LCII_handle_fin(LCII_packet_t* packet)
{
  LCII_context_t* ctx;
  memcpy(&ctx, packet->data.address, sizeof(ctx));
  LCI_DBG_Log(LCI_LOG_TRACE, "rdv", "recv FIN: rctx %p\n", ctx);
  LCII_free_packet(packet);
  LCII_handle_rdv_remote_comp(ctx);
}

static inline void LCII_handle_writeImm(LCI_endpoint_t ep, uint64_t ctx_key)
{
  LCII_context_t* ctx =
      (LCII_context_t*)LCM_archive_remove(ep->ctx_archive_p, ctx_key);
  LCI_DBG_Log(LCI_LOG_TRACE, "rdv",
              "complete recvl: ctx %p rank %d "
              "tag %d user_ctx %p completion attr %x completion %p\n",
              ctx, ctx->rank, ctx->tag, ctx->user_context, ctx->comp_attr,
              ctx->completion);
  LCII_handle_rdv_remote_comp(ctx);
}

#endif  // LCI_RENDEZVOUS_H
