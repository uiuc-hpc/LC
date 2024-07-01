#include "lci.h"
#include "runtime/lcii.h"

LCI_error_t LCI_puts(LCI_endpoint_t ep, LCI_short_t src, int rank,
                     LCI_tag_t tag, uintptr_t remote_completion)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(remote_completion == LCI_DEFAULT_COMP_REMOTE,
                 "Only support default remote completion "
                 "(set by LCI_plist_set_default_comp, "
                 "the default value is LCI_UR_CQ)\n");
  LCI_error_t ret = LCIS_post_sends(
      ep->device->endpoint_worker->endpoint, rank, &src, sizeof(LCI_short_t),
      LCII_MAKE_PROTO(ep->gid, LCI_MSG_RDMA_SHORT, tag));
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(put, sizeof(LCI_short_t));
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_puts(ep %p, rank %d, tag %u, remote_completion %p) -> %d\n",
              ep, rank, tag, (void*)remote_completion, ret);
  return ret;
}

LCI_error_t LCI_putm(LCI_endpoint_t ep, LCI_mbuffer_t mbuffer, int rank,
                     LCI_tag_t tag, LCI_lbuffer_t rbuffer,
                     uintptr_t remote_completion)
{
  //  LC_POOL_GET_OR_RETN(ep->pkpool, p);
  //  lc_pk_init(ep, (size > 1024) ? lc_pool_get_local(ep->pkpool) : -1,
  //  LC_PROTO_DATA, p); struct lc_rep* rep = &(ep->rep[rank]);
  //  memcpy(p->data.buffer, src, size);
  //  LCIS_post_put(ep->server, rep->handle, rep->base, offset, rep->rkey, size,
  //  LCII_MAKE_PROTO(ep->gid, LC_PROTO_LONG, meta), p);
  return LCI_ERR_FEATURE_NA;
}

LCI_error_t LCI_putmac(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, uintptr_t remote_completion,
                       LCI_comp_t local_completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(buffer.length <= LCI_MEDIUM_SIZE,
                 "buffer is too large %lu (maximum: %d)\n", buffer.length,
                 LCI_MEDIUM_SIZE);
  LCI_DBG_Assert(remote_completion == LCI_DEFAULT_COMP_REMOTE,
                 "Only support default remote completion "
                 "(set by LCI_plist_set_default_comp, "
                 "the default value is LCI_UR_CQ)\n");
  LCI_error_t ret = LCI_OK;
  bool is_user_provided_packet =
      LCII_is_packet(ep->device->heap, buffer.address);
  if (local_completion == NULL && buffer.length <= LCI_SHORT_SIZE) {
    /* if data is this short, we will be able to inline it
     * no reason to get a packet, allocate a ctx, etc */
    ret = LCIS_post_sends(ep->device->endpoint_worker->endpoint, rank,
                          buffer.address, buffer.length,
                          LCII_MAKE_PROTO(ep->gid, LCI_MSG_RDMA_MEDIUM, tag));
    if (ret == LCI_OK && is_user_provided_packet) {
      LCII_packet_t* packet = LCII_mbuffer2packet(buffer);
      packet->context.poolid = -1;
      LCII_free_packet(packet);
    }
  } else {
    LCII_packet_t* packet;
    if (is_user_provided_packet) {
      packet = LCII_mbuffer2packet(buffer);
    } else {
      packet = LCII_alloc_packet_nb(ep->pkpool);
      if (packet == NULL) {
        // no packet is available
        return LCI_ERR_RETRY;
      }
      memcpy(packet->data.address, buffer.address, buffer.length);
    }
    packet->context.poolid = (buffer.length > LCI_PACKET_RETURN_THRESHOLD)
                                 ? lc_pool_get_local(ep->pkpool)
                                 : -1;

    LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
    ctx->data.packet = packet;
    LCII_initilize_comp_attr(ctx->comp_attr);
    if (!(is_user_provided_packet && local_completion)) {
      LCII_comp_attr_set_free_packet(ctx->comp_attr, 1);
    }
    if (local_completion) {
      ctx->data_type = LCI_MEDIUM;
      ctx->data.mbuffer = buffer;
      ctx->rank = rank;
      ctx->tag = tag;
      ctx->user_context = user_context;
      LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->cmd_comp_type);
      ctx->completion = local_completion;
    }
    ret = LCIS_post_send(
        ep->device->endpoint_worker->endpoint, rank, packet->data.address,
        buffer.length, ep->device->heap_segment->mr,
        LCII_MAKE_PROTO(ep->gid, LCI_MSG_RDMA_MEDIUM, tag), ctx);
    if (ret == LCI_ERR_RETRY) {
      if (!is_user_provided_packet) LCII_free_packet(packet);
      LCIU_free(ctx);
    }
  }
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(put, (int64_t)buffer.length);
  }
  LCI_DBG_Log(
      LCI_LOG_TRACE, "comm",
      "LCI_putmac(ep %p, buffer {%p, %lu}, rank %d, tag %u, "
      "remote_completion %p, local_completion %p, user_context %p) -> %d\n",
      ep, buffer.address, buffer.length, rank, tag, (void*)remote_completion,
      local_completion, user_context, ret);
  return ret;
}

LCI_error_t LCI_putma(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag, uintptr_t remote_completion)
{
  return LCI_putmac(ep, buffer, rank, tag, remote_completion, NULL, NULL);
}

LCI_error_t LCI_putmna(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, uintptr_t remote_completion)
{
  return LCI_putmac(ep, buffer, rank, tag, remote_completion, NULL, NULL);
}

LCI_error_t LCI_putl(LCI_endpoint_t ep, LCI_lbuffer_t local_buffer,
                     LCI_comp_t local_completion, int rank, LCI_tag_t tag,
                     LCI_lbuffer_t rbuffer, uintptr_t remote_completion)
{
  return LCI_ERR_FEATURE_NA;
}

LCI_error_t LCI_putla(LCI_endpoint_t ep, LCI_lbuffer_t buffer,
                      LCI_comp_t completion, int rank, LCI_tag_t tag,
                      uintptr_t remote_completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(remote_completion == LCI_DEFAULT_COMP_REMOTE,
                 "Only support default remote completion "
                 "(set by LCI_plist_set_default_comp, "
                 "the default value is LCI_UR_CQ)\n");
  if (!LCII_bq_is_empty(ep->bq_p)) {
    return LCI_ERR_RETRY;
  }
  LCII_packet_t* packet = LCII_alloc_packet_nb(ep->pkpool);
  if (packet == NULL) {
    // no packet is available
    return LCI_ERR_RETRY;
  }
  packet->context.poolid = LCII_POOLID_LOCAL;

  LCII_context_t* rts_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rts_ctx->data.packet = packet;
  LCII_initilize_comp_attr(rts_ctx->comp_attr);
  LCII_comp_attr_set_free_packet(rts_ctx->comp_attr, 1);

  LCII_context_t* rdv_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rdv_ctx->data.lbuffer = buffer;
  rdv_ctx->data_type = LCI_LONG;
  rdv_ctx->rank = rank;
  rdv_ctx->tag = tag;
  rdv_ctx->user_context = user_context;
  LCII_initilize_comp_attr(rdv_ctx->comp_attr);
  LCII_comp_attr_set_rdv_type(rdv_ctx->comp_attr, LCII_RDV_1SIDED);
  LCII_comp_attr_set_comp_type(rdv_ctx->comp_attr, ep->cmd_comp_type);
  LCII_comp_attr_set_dereg(rdv_ctx->comp_attr,
                           buffer.segment == LCI_SEGMENT_ALL);
  rdv_ctx->completion = completion;

  packet->data.rts.rdv_type = LCII_RDV_1SIDED;
  packet->data.rts.send_ctx = (uintptr_t)rdv_ctx;
  packet->data.rts.size = buffer.length;

  LCI_DBG_Log(LCI_LOG_TRACE, "rdv", "send rts: type %d sctx %p size %lu\n",
              packet->data.rts.rdv_type, (void*)packet->data.rts.send_ctx,
              packet->data.rts.size);
  LCI_error_t ret = LCIS_post_send(
      ep->device->endpoint_worker->endpoint, rank, packet->data.address,
      sizeof(struct LCII_packet_rts_t), ep->device->heap_segment->mr,
      LCII_MAKE_PROTO(ep->gid, LCI_MSG_RTS, tag), rts_ctx);
  if (ret == LCI_ERR_RETRY) {
    LCII_free_packet(packet);
    LCIU_free(rts_ctx);
    LCIU_free(rdv_ctx);
  }
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(put, (int64_t)buffer.length);
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_putla(ep %p, buffer {%p, %lu, %p}, completion %p, rank %d, "
              "tag %u, remote_completion %p, user_context %p) -> %d\n",
              ep, buffer.address, buffer.length, buffer.segment, completion,
              rank, tag, (void*)remote_completion, user_context, ret);
  return ret;
}

LCI_error_t LCI_putva(LCI_endpoint_t ep, LCI_iovec_t iovec,
                      LCI_comp_t completion, int rank, LCI_tag_t tag,
                      uintptr_t remote_completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(remote_completion == LCI_DEFAULT_COMP_REMOTE,
                 "Only support default remote completion "
                 "(set by LCI_plist_set_default_comp, "
                 "the default value is LCI_UR_CQ)\n");
  LCI_DBG_Assert(iovec.count > 0, "iovec.count = %d!\n", iovec.count);
  LCI_DBG_Assert(iovec.count <= LCI_IOVEC_SIZE,
                 "iovec.count = %d > "
                 "LCI_IOVEC_SIZE %d!\n",
                 iovec.count, LCI_IOVEC_SIZE);
  LCI_DBG_Assert(
      iovec.piggy_back.length <= LCI_get_iovec_piggy_back_size(iovec.count),
      "iovec's piggy back is too large! (%lu > %lu)\n", iovec.piggy_back.length,
      LCI_get_iovec_piggy_back_size(iovec.count));
  for (int i = 0; i < iovec.count; ++i) {
    LCI_DBG_Assert(
        (iovec.lbuffers[0].segment == LCI_SEGMENT_ALL &&
         iovec.lbuffers[i].segment == LCI_SEGMENT_ALL) ||
            (iovec.lbuffers[0].segment != LCI_SEGMENT_ALL &&
             iovec.lbuffers[i].segment != LCI_SEGMENT_ALL),
        "We currently require either all lbuffers to be registers or "
        "all of them are LCI_SEGMENT_ALL\n");
    LCI_DBG_Assert(iovec.lbuffers[i].length > 0, "Invalid lbuffer length\n");
  }
  if (!LCII_bq_is_empty(ep->bq_p)) {
    return LCI_ERR_RETRY;
  }
  LCII_packet_t* packet = LCII_alloc_packet_nb(ep->pkpool);
  if (packet == NULL) {
    // no packet is available
    return LCI_ERR_RETRY;
  }
  packet->context.poolid =
      (iovec.piggy_back.length > LCI_PACKET_RETURN_THRESHOLD)
          ? lc_pool_get_local(ep->pkpool)
          : -1;

  LCII_context_t* rts_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rts_ctx->data.packet = packet;
  LCII_initilize_comp_attr(rts_ctx->comp_attr);
  LCII_comp_attr_set_free_packet(rts_ctx->comp_attr, 1);

  LCII_context_t* rdv_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rdv_ctx->data.iovec = iovec;
  rdv_ctx->data_type = LCI_IOVEC;
  rdv_ctx->rank = rank;
  rdv_ctx->tag = tag;
  rdv_ctx->user_context = user_context;
  LCII_initilize_comp_attr(rdv_ctx->comp_attr);
  LCII_comp_attr_set_rdv_type(rdv_ctx->comp_attr, LCII_RDV_IOVEC);
  LCII_comp_attr_set_comp_type(rdv_ctx->comp_attr, ep->cmd_comp_type);
  // Currently, for iovec, if one buffer uses LCI_SEGMENT_ALL,
  // all buffers need to use LCI_SEGMENT_ALL
  LCII_comp_attr_set_dereg(rdv_ctx->comp_attr,
                           iovec.lbuffers[0].segment == LCI_SEGMENT_ALL);
  rdv_ctx->completion = completion;

  packet->data.rts.rdv_type = LCII_RDV_IOVEC;
  packet->data.rts.send_ctx = (uintptr_t)rdv_ctx;
  packet->data.rts.count = iovec.count;
  packet->data.rts.piggy_back_size = iovec.piggy_back.length;
  for (int i = 0; i < iovec.count; ++i) {
    packet->data.rts.size_p[i] = iovec.lbuffers[i].length;
  }
  memcpy((void*)&packet->data.rts.size_p[iovec.count], iovec.piggy_back.address,
         iovec.piggy_back.length);

  LCI_DBG_Log(LCI_LOG_TRACE, "rdv",
              "send rts: type %d sctx %p count %d "
              "piggy_back_size %lu\n",
              packet->data.rts.rdv_type, (void*)packet->data.rts.send_ctx,
              packet->data.rts.count, packet->data.rts.piggy_back_size);
  size_t length = (uintptr_t)&packet->data.rts.size_p[iovec.count] -
                  (uintptr_t)packet->data.address + iovec.piggy_back.length;
  LCI_error_t ret =
      LCIS_post_send(ep->device->endpoint_worker->endpoint, rank,
                     packet->data.address, length, ep->device->heap_segment->mr,
                     LCII_MAKE_PROTO(ep->gid, LCI_MSG_RTS, tag), rts_ctx);
  if (ret == LCI_ERR_RETRY) {
    LCII_free_packet(packet);
    LCIU_free(rts_ctx);
    LCIU_free(rdv_ctx);
  }
  if (ret == LCI_OK) {
    uint64_t total_length = iovec.piggy_back.length;
    for (int i = 0; i < iovec.count; ++i) {
      total_length += iovec.lbuffers[i].length;
    }
    LCII_PCOUNTER_ADD(put, (int64_t)total_length);
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_putva(ep %p, iovec {{%p, %lu}, %d, %p}, completion %p, rank "
              "%d, tag %u, remote_completion %p, user_context %p) -> %d\n",
              ep, iovec.piggy_back.address, iovec.piggy_back.length,
              iovec.count, iovec.lbuffers, completion, rank, tag,
              (void*)remote_completion, user_context, ret);
  return ret;
}

size_t LCI_get_iovec_piggy_back_size(int count)
{
  LCI_DBG_Assert(LCI_MEDIUM_SIZE > 0,
                 "LCI_MEDIUM_SIZE <=0! You should run "
                 "LCI_initialize() before calling this function\n");
  LCI_DBG_Assert(LCI_MEDIUM_SIZE - sizeof(struct LCII_packet_rts_t) >=
                     sizeof(size_t) * count,
                 "Too many lbuffers to send in one iovec!\n");
  return LCI_MEDIUM_SIZE - sizeof(struct LCII_packet_rts_t) -
         sizeof(size_t) * count;
}
