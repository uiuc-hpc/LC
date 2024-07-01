#ifndef PACKET_H_
#define PACKET_H_

#define LCII_POOLID_LOCAL (-1)
struct __attribute__((packed)) LCII_packet_context {
  // Most of the current ctx requires 128-bits (FIXME)
  int64_t hwctx[2];
  // Here is LCI context.
  LCII_pool_t* pkpool; /* the packet pool this packet belongs to */
  int poolid;          /* id of the pool to return this packet.
                        * -1 means returning to the local pool */
  int src_rank;
  int length; /* length for its payload */
#ifdef LCI_DEBUG
  bool isInPool; /* Debug use only. Whether this packet is in the packet pool */
#endif
};

struct __attribute__((packed)) LCII_packet_rts_t {
  uintptr_t
      send_ctx; /* the address of the related context on the source side */
  LCII_rdv_type_t rdv_type; /* type of this rendezvous message */
  union {
    // for LCI_LONG
    size_t size;
    // for LCI_IOVEC
    struct {
      int count;
      size_t piggy_back_size;
      size_t size_p[0];
    };
  };
};

struct __attribute__((packed)) LCII_packet_rtr_rbuffer_info_t {
  LCIS_rkey_t rkey;
  uintptr_t remote_addr_base;
  LCIS_offset_t remote_addr_offset;
};

struct __attribute__((packed)) LCII_packet_rtr_t {
  uintptr_t
      send_ctx; /* the address of the related context on the source side */
  LCII_rdv_type_t rdv_type; /* type of this rendezvous message */
  union {
    // When using writeimm protocol
    uint32_t
        recv_ctx_key; /* the id of the related context on the target side */
    // when using write protocol
    uintptr_t recv_ctx;
  };
  struct LCII_packet_rtr_rbuffer_info_t rbuffer_info_p[0];
};

struct __attribute__((packed, aligned(LCI_CACHE_LINE))) LCII_packet_data_t {
  union {
    struct LCII_packet_rts_t rts;
    struct LCII_packet_rtr_t rtr;
    char address[0];
  };
};

typedef struct __attribute__((packed)) LCII_packet_t {
  struct LCII_packet_context context;
  struct LCII_packet_data_t data;
} LCII_packet_t;

static inline LCII_packet_t* LCII_alloc_packet_nb(struct LCII_pool_t* pool)
{
  LCII_packet_t* packet = LCII_pool_get_nb(pool);
  if (packet != NULL) {
    LCII_PCOUNTER_ADD(packet_get, 1);
    packet->context.poolid = LCII_POOLID_LOCAL;
#ifdef LCI_DEBUG
    LCI_DBG_Assert(packet->context.isInPool,
                   "This packet has already been allocated!\n");
    packet->context.isInPool = false;
#endif
  }
  return packet;
}

static inline void LCII_free_packet(LCII_packet_t* packet)
{
  LCI_DBG_Assert(((uint64_t)packet + sizeof(struct LCII_packet_context)) %
                         LCI_CACHE_LINE ==
                     0,
                 "Not a packet (address %p)!\n", packet);
#ifdef LCI_DEBUG
  LCI_DBG_Assert(!packet->context.isInPool,
                 "This packet has already been freed!\n");
  packet->context.isInPool = true;
#endif
  LCII_PCOUNTER_ADD(packet_put, 1);
  if (packet->context.poolid != LCII_POOLID_LOCAL)
    LCII_pool_put_to(packet->context.pkpool, packet, packet->context.poolid);
  else
    LCII_pool_put(packet->context.pkpool, packet);
}

static inline LCII_packet_t* LCII_mbuffer2packet(LCI_mbuffer_t mbuffer)
{
  return (LCII_packet_t*)(mbuffer.address - offsetof(LCII_packet_t, data));
}

static inline bool LCII_is_packet(LCII_packet_heap_t* heap, void* address)
{
  void* packet_address =
      (LCII_packet_t*)(address - offsetof(LCII_packet_t, data));
  uintptr_t offset = (uintptr_t)packet_address - (uintptr_t)heap->base_packet_p;
  return (uintptr_t)packet_address >= (uintptr_t)heap->base_packet_p &&
         offset % LCI_PACKET_SIZE == 0 &&
         offset / LCI_PACKET_SIZE < LCI_SERVER_NUM_PKTS;
}

#endif
