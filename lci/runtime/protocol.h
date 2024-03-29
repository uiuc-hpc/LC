#ifndef LC_PROTO_H
#define LC_PROTO_H

// 32 bits for rank, 16 bits for endpoint ID, 16 bits for tag
static inline uint64_t LCII_make_key(LCI_endpoint_t ep, int rank, LCI_tag_t tag)
{
  uint64_t ret = 0;
  if (ep->match_type == LCI_MATCH_RANKTAG) {
    ret = LCIU_set_bits64(0, rank, 32, 32) |
          LCIU_set_bits64(0, ep->gid, 16, 16) | LCIU_set_bits64(0, tag, 16, 0);
  } else {
    LCI_DBG_Assert(ep->match_type == LCI_MATCH_TAG, "Unknown match_type %d\n",
                   ep->match_type);
    const uint32_t rank_any = -1;
    ret = LCIU_set_bits64(0, rank_any, 32, 32) |
          LCIU_set_bits64(0, ep->gid, 16, 16) | LCIU_set_bits64(0, tag, 16, 0);
  }
  return ret;
}

static inline void lc_ce_dispatch(LCII_context_t* ctx)
{
  if (LCII_comp_attr_get_dereg(ctx->comp_attr) == 1) {
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
      LCI_memory_deregister(&lbuffer->segment);
      lbuffer->segment = LCI_SEGMENT_ALL;
    }
  }
  switch (LCII_comp_attr_get_comp_type(ctx->comp_attr)) {
    case LCI_COMPLETION_NONE:
      LCIU_free(ctx);
      break;
#ifdef LCI_SERVER_HAS_SYNC
    case LCI_COMPLETION_SYNC: {
      LCII_sync_signal(ctx->completion, ctx);
      break;
    }
#endif
#ifdef LCI_SERVER_HAS_CQ
    case LCI_COMPLETION_QUEUE:
      LCII_queue_push(ctx->completion, ctx);
      break;
#endif
#ifdef LCI_SERVER_HAS_AM
    case LCI_COMPLETION_HANDLER: {
      LCI_handler_t handler = ctx->completion;
      LCII_PCOUNTER_ADD(comp_produce, 1);
      LCII_PCOUNTER_ADD(comp_consume, 1);
      (*handler)(LCII_ctx2req(ctx));
      break;
    }
#endif
    default:
      LCI_DBG_Assert(false, "Unknown completion type: %d!\n",
                     (int)LCII_comp_attr_get_comp_type(ctx->comp_attr));
  }
}

static inline void LCIS_serve_recv(void* p, int src_rank, size_t length,
                                   uint32_t imm_data)
{
  LCII_PCOUNTER_ADD(net_recv_comp, length);
  LCII_packet_t* packet = (LCII_packet_t*)p;
  LCII_proto_t proto = imm_data;
  // NOTE: this should be RGID because it is received from remote.
  LCI_endpoint_t ep = LCI_ENDPOINTS[PROTO_GET_RGID(proto)];
  LCI_tag_t tag = PROTO_GET_TAG(proto);
  LCI_msg_type_t msg_type = PROTO_GET_TYPE(proto);
  packet->context.src_rank = src_rank;

  switch (msg_type) {
    case LCI_MSG_SHORT: {
      LCI_DBG_Assert(length == LCI_SHORT_SIZE,
                     "Unexpected message length %lu\n", length);
      uint64_t key = LCII_make_key(ep, src_rank, tag);
      uint64_t value = (uint64_t)packet;
      if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_SEND) ==
          LCI_OK) {
        LCII_context_t* ctx = (LCII_context_t*)value;
        // If the receiver uses LCI_MATCH_TAG, we have to set the rank here.
        ctx->rank = src_rank;
        memcpy(&(ctx->data.immediate), packet->data.address, LCI_SHORT_SIZE);
        LCII_free_packet(packet);
        lc_ce_dispatch(ctx);
      }
      break;
    }
    case LCI_MSG_MEDIUM: {
      uint64_t key = LCII_make_key(ep, src_rank, tag);
      uint64_t value = (uint64_t)packet;
      packet->context.length = length;
      if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_SEND) ==
          LCI_OK) {
        LCII_context_t* ctx = (LCII_context_t*)value;
        ctx->rank = src_rank;
        ctx->data.mbuffer.length = length;
        if (ctx->data.mbuffer.address != NULL) {
          // copy to user provided buffer
          memcpy(ctx->data.mbuffer.address, packet->data.address,
                 ctx->data.mbuffer.length);
          LCII_free_packet(packet);
        } else {
          // use LCI packet
          ctx->data.mbuffer.address = packet->data.address;
        }
        lc_ce_dispatch(ctx);
      }
      break;
    }
    case LCI_MSG_RTS: {
      LCII_PCOUNTER_START(serve_rts_timer);
      if (packet->data.rts.rdv_type == LCII_RDV_2SIDED) {
        const uint64_t key = LCII_make_key(ep, src_rank, tag);
        uint64_t value = (uint64_t)packet;
        if (LCII_matchtable_insert(ep->mt, key, &value, LCII_MATCHTABLE_SEND) ==
            LCI_OK) {
          LCII_handle_rts(ep, packet, src_rank, tag, (LCI_comp_t)value, true);
        }
      } else {
        LCII_handle_rts(ep, packet, src_rank, tag, NULL, true);
      }
      LCII_PCOUNTER_END(serve_rts_timer);
      break;
    }
    case LCI_MSG_RTR: {
      LCII_PCOUNTER_START(serve_rtr_timer);
      LCII_handle_rtr(ep, packet);
      LCII_PCOUNTER_END(serve_rtr_timer);
      break;
    }
    case LCI_MSG_RDMA_SHORT: {
      // dynamic put
      LCI_DBG_Assert(length == LCI_SHORT_SIZE,
                     "Unexpected message length %lu\n", length);
      LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
      memcpy(&(ctx->data.immediate), packet->data.address, LCI_SHORT_SIZE);
      LCII_free_packet(packet);
      ctx->data_type = LCI_IMMEDIATE;
      ctx->rank = src_rank;
      ctx->tag = tag;
      LCII_initilize_comp_attr(ctx->comp_attr);
      LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->msg_comp_type);
      ctx->completion = ep->default_comp;
      ctx->user_context = NULL;
      lc_ce_dispatch(ctx);
      break;
    }
    case LCI_MSG_RDMA_MEDIUM: {
      LCII_context_t* ctx = LCIU_malloc(sizeof(LCII_context_t));
      ctx->data.mbuffer.address = packet->data.address;
      ctx->data.mbuffer.length = length;
      ctx->data_type = LCI_MEDIUM;
      ctx->rank = src_rank;
      ctx->tag = tag;
      LCII_initilize_comp_attr(ctx->comp_attr);
      LCII_comp_attr_set_comp_type(ctx->comp_attr, ep->msg_comp_type);
      ctx->completion = ep->default_comp;
      ctx->user_context = NULL;
      lc_ce_dispatch(ctx);
      break;
    }
    case LCI_MSG_FIN: {
      LCI_DBG_Assert(length == sizeof(LCII_context_t*),
                     "Unexpected FIN message length (%lu)!\n", length);
      LCII_handle_fin(packet);
      break;
    }
    default:
      LCI_Assert(false, "Unknown proto!\n");
  }
}

static inline void LCIS_serve_rdma(uint32_t imm_data)
{
  LCII_PCOUNTER_START(serve_rdma_timer);
  LCII_proto_t proto = imm_data;
  LCI_endpoint_t ep = LCI_ENDPOINTS[PROTO_GET_RGID(proto)];
  uint16_t tag = PROTO_GET_TAG(proto);
  LCI_msg_type_t msg_type = PROTO_GET_TYPE(proto);
  LCI_DBG_Assert(msg_type == LCI_MSG_RDV_DATA, "");

  LCII_handle_writeImm(ep, tag);
  LCII_PCOUNTER_END(serve_rdma_timer);
}

// local completion
static inline void LCIS_serve_send(void* raw_ctx)
{
  LCII_context_t* ctx = raw_ctx;
  if (LCII_comp_attr_get_extended(ctx->comp_attr) == 1) {
    // handle extended context
    LCII_handle_rdv_data_local_comp((LCII_extended_context_t*)ctx);
    return;
  }
  if (LCII_comp_attr_get_free_packet(ctx->comp_attr) == 1) {
    LCII_free_packet(ctx->data.packet);
  }
  lc_ce_dispatch(ctx);
}

#endif
