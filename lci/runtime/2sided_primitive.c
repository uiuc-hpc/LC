#include "lci.h"
#include "runtime/lcii.h"

LCI_error_t LCI_sends(LCI_endpoint_t ep, LCI_short_t src, int rank,
                      LCI_tag_t tag)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_error_t ret = LCIS_post_sends(
      ep->device->endpoint_worker->endpoint, rank, &src, sizeof(LCI_short_t),
      LCII_MAKE_PROTO(ep->gid, LCI_MSG_SHORT, tag));
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(send, sizeof(LCI_short_t));
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_sends(ep %p, rank %d, tag %u) -> %d\n", ep, rank, tag, ret);
  return ret;
}

LCI_error_t LCI_sendmc(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(buffer.length <= LCI_MEDIUM_SIZE,
                 "buffer is too large %lu (maximum: %d)\n", buffer.length,
                 LCI_MEDIUM_SIZE);
  LCI_error_t ret = LCI_OK;
  bool is_user_provided_packet =
      LCII_is_packet(ep->device->heap, buffer.address);
  if (completion == NULL && buffer.length <= LCI_SHORT_SIZE) {
    /* if data is this short, we will be able to inline it
     * no reason to get a packet, allocate a ctx, etc */
    ret = LCIS_post_sends(ep->device->endpoint_worker->endpoint, rank,
                          buffer.address, buffer.length,
                          LCII_MAKE_PROTO(ep->gid, LCI_MSG_MEDIUM, tag));
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
    if (!(is_user_provided_packet && completion)) {
      LCII_comp_attr_set_free_packet(ctx->comp_attr, 1);
    }
    if (completion) {
      ctx->data_type = LCI_MEDIUM;
      ctx->data.mbuffer = buffer;
      ctx->rank = rank;
      ctx->tag = tag;
      ctx->user_context = user_context;
      LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->cmd_comp_type);
      ctx->completion = completion;
    }

    ret = LCIS_post_send(ep->device->endpoint_worker->endpoint, rank,
                         packet->data.address, buffer.length,
                         ep->device->heap_segment->mr,
                         LCII_MAKE_PROTO(ep->gid, LCI_MSG_MEDIUM, tag), ctx);
    if (ret == LCI_ERR_RETRY) {
      if (!is_user_provided_packet) LCII_free_packet(packet);
      LCIU_free(ctx);
    }
  }
  if (ret == LCI_OK) {
    LCII_PCOUNTER_ADD(send, (int64_t)buffer.length);
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_sendmc(ep %p, buffer {%p, %lu}(%d), rank %d, tag %u, "
              "completion %p, user_context %p) -> %d\n",
              ep, buffer.address, buffer.length, is_user_provided_packet, rank,
              tag, ret, completion, user_context);
  return ret;
}

LCI_error_t LCI_sendm(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag)
{
  return LCI_sendmc(ep, buffer, rank, tag, NULL, NULL);
}

LCI_error_t LCI_sendmn(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag)
{
  return LCI_sendmc(ep, buffer, rank, tag, NULL, NULL);
}

LCI_error_t LCI_sendl(LCI_endpoint_t ep, LCI_lbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
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
  LCII_comp_attr_set_comp_type(rdv_ctx->comp_attr, ep->cmd_comp_type);
  LCII_comp_attr_set_dereg(rdv_ctx->comp_attr,
                           buffer.segment == LCI_SEGMENT_ALL);
  LCII_comp_attr_set_rdv_type(rdv_ctx->comp_attr, LCII_RDV_2SIDED);
  rdv_ctx->completion = completion;

  packet->data.rts.rdv_type = LCII_RDV_2SIDED;
  packet->data.rts.send_ctx = (uintptr_t)rdv_ctx;
  packet->data.rts.size = buffer.length;

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
    LCII_PCOUNTER_ADD(send, (int64_t)buffer.length);
  }
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_sendl(ep %p, buffer {%p, %lu, %p}, rank %d, tag %u, "
              "completion %p, user_context %p) -> %d\n",
              ep, buffer.address, buffer.length, buffer.segment, rank, tag,
              completion, user_context, ret);
  return ret;
}

LCI_error_t LCI_recvs(LCI_endpoint_t ep, int rank, LCI_tag_t tag,
                      LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
  ctx->data_type = LCI_IMMEDIATE;
  ctx->rank = rank;
  ctx->tag = tag;
  ctx->user_context = user_context;
  LCII_initilize_comp_attr(ctx->comp_attr);
  LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->msg_comp_type);
  ctx->completion = completion;

  uint64_t key = LCII_make_key(ep, rank, tag);
  uint64_t value = (uint64_t)ctx;
  if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_RECV) ==
      LCI_OK) {
    LCII_packet_t* packet = (LCII_packet_t*)value;
    ctx->rank = packet->context.src_rank;
    memcpy(&(ctx->data.immediate), packet->data.address, LCI_SHORT_SIZE);
    LCII_free_packet(packet);
    lc_ce_dispatch(ctx);
  }
  LCII_PCOUNTER_ADD(recv, 1);
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_recvs(ep %p, rank %d, tag %u, completion %p, user_context "
              "%p) -> %d\n",
              ep, rank, tag, completion, user_context, LCI_OK);
  return LCI_OK;
}

LCI_error_t LCI_recvm(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCI_DBG_Assert(buffer.length <= LCI_MEDIUM_SIZE,
                 "buffer is too large %lu (maximum: %d)\n", buffer.length,
                 LCI_MEDIUM_SIZE);
  LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
  ctx->data.mbuffer = buffer;
  ctx->data_type = LCI_MEDIUM;
  ctx->rank = rank;
  ctx->tag = tag;
  ctx->user_context = user_context;
  LCII_initilize_comp_attr(ctx->comp_attr);
  LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->msg_comp_type);
  ctx->completion = completion;

  uint64_t key = LCII_make_key(ep, rank, tag);
  uint64_t value = (uint64_t)ctx;
  if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_RECV) ==
      LCI_OK) {
    LCII_packet_t* packet = (LCII_packet_t*)value;
    ctx->rank = packet->context.src_rank;
    ctx->data.mbuffer.length = packet->context.length;
    // copy to user provided buffer
    memcpy(ctx->data.mbuffer.address, packet->data.address,
           ctx->data.mbuffer.length);
    LCII_free_packet(packet);
    lc_ce_dispatch(ctx);
  }
  LCII_PCOUNTER_ADD(recv, 1);
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_recvm(ep %p, buffer {%p, %lu}, rank %d, tag %u, completion "
              "%p, user_context %p) -> %d\n",
              ep, buffer.address, buffer.length, rank, tag, completion,
              user_context, LCI_OK);
  return LCI_OK;
}

LCI_error_t LCI_recvmn(LCI_endpoint_t ep, int rank, LCI_tag_t tag,
                       LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
  ctx->data.mbuffer.address = NULL;
  ctx->data_type = LCI_MEDIUM;
  ctx->rank = rank;
  ctx->tag = tag;
  ctx->user_context = user_context;
  LCII_initilize_comp_attr(ctx->comp_attr);
  LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->msg_comp_type);
  ctx->completion = completion;

  uint64_t key = LCII_make_key(ep, rank, tag);
  uint64_t value = (uint64_t)ctx;
  if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_RECV) ==
      LCI_OK) {
    LCII_packet_t* packet = (LCII_packet_t*)value;
    ctx->rank = packet->context.src_rank;
    ctx->data.mbuffer.length = packet->context.length;
    // use LCI packet
    ctx->data.mbuffer.address = packet->data.address;
    lc_ce_dispatch(ctx);
  }
  LCII_PCOUNTER_ADD(recv, 1);
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_recvmn(ep %p, rank %d, tag %u, completion %p, user_context "
              "%p) -> %d\n",
              ep, rank, tag, completion, user_context, LCI_OK);
  return LCI_OK;
}

LCI_error_t LCI_recvl(LCI_endpoint_t ep, LCI_lbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context)
{
  LCI_DBG_Assert(tag <= LCI_MAX_TAG, "tag %d is too large (maximum: %d)\n", tag,
                 LCI_MAX_TAG);
  LCII_context_t* rdv_ctx = LCIU_malloc(sizeof(LCII_context_t));
  rdv_ctx->data.lbuffer = buffer;
  rdv_ctx->data_type = LCI_LONG;
  rdv_ctx->rank = rank;
  rdv_ctx->tag = tag;
  rdv_ctx->user_context = user_context;
  LCII_initilize_comp_attr(rdv_ctx->comp_attr);
  LCII_comp_attr_set_comp_type(rdv_ctx->comp_attr, ep->msg_comp_type);
  LCII_comp_attr_set_dereg(
      rdv_ctx->comp_attr,
      buffer.address != NULL && buffer.segment == LCI_SEGMENT_ALL);
  LCII_comp_attr_set_rdv_type(rdv_ctx->comp_attr, LCII_RDV_2SIDED);
  rdv_ctx->completion = completion;

  uint64_t key = LCII_make_key(ep, rank, tag);
  uint64_t value = (uint64_t)rdv_ctx;
  if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_RECV) ==
      LCI_OK) {
    LCII_packet_t* packet = (LCII_packet_t*)value;
    LCII_handle_rts(ep, packet, packet->context.src_rank, tag, rdv_ctx, false);
  }
  LCII_PCOUNTER_ADD(recv, 1);
  LCI_DBG_Log(LCI_LOG_TRACE, "comm",
              "LCI_recvl(ep %p, buffer {%p, %lu, %p}, rank %d, tag %u, "
              "completion %p, user_context %p) -> %d\n",
              ep, buffer.address, buffer.length, buffer.segment, rank, tag,
              completion, user_context, LCI_OK);
  return LCI_OK;
}
