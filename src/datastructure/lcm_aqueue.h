#ifndef LCI_LCM_AQUEUE_H
#define LCI_LCM_AQUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LCM_aqueue_entry_t {
  atomic_uint_least64_t data;
  LCIU_CACHE_PADDING(sizeof(atomic_uint_least64_t));
} LCM_aqueue_entry_t;

typedef struct LCM_aqueue_t {
  atomic_uint_fast64_t top;  // point to the next entry that is empty
  LCIU_CACHE_PADDING(sizeof(atomic_uint_fast64_t));
  atomic_uint_fast64_t bot;  // point to the fist entry that is full
  LCIU_CACHE_PADDING(sizeof(atomic_uint_fast64_t));
  uint_fast64_t length;
  struct LCM_aqueue_entry_t* container;  // a pointer to type void*
} LCM_aqueue_t;

// The following functions are not thread-safe
static inline void LCM_aqueue_init(LCM_aqueue_t* queue, uint_fast64_t capacity);
static inline void LCM_aqueue_fina(LCM_aqueue_t* queue);
// The following functions are thread-safe
static inline void LCM_aqueue_push(LCM_aqueue_t* queue, void* val);
static inline void* LCM_aqueue_pop(LCM_aqueue_t* queue);

#ifdef __cplusplus
}
#endif

static inline void LCM_aqueue_init(LCM_aqueue_t* queue, uint_fast64_t capacity)
{
  LCM_Assert(sizeof(LCM_aqueue_entry_t) == LCI_CACHE_LINE,
             "Unexpected sizeof(LCM_aqueue_entry_t) %lu\n",
             sizeof(LCM_aqueue_entry_t));
  queue->container = LCIU_memalign(LCI_CACHE_LINE,
                                   (capacity + 1) * sizeof(LCM_aqueue_entry_t));
  atomic_init(&queue->top, 0);
  atomic_init(&queue->bot, 0);
  queue->length = capacity + 1;
  for (int i = 0; i < queue->length; ++i) {
    atomic_init(&queue->container[i].data, 0);
  }
  atomic_thread_fence(LCIU_memory_order_seq_cst);
}

static inline void LCM_aqueue_fina(LCM_aqueue_t* queue)
{
  atomic_thread_fence(LCIU_memory_order_seq_cst);
  LCIU_free(queue->container);
  queue->container = NULL;
  atomic_init(&queue->top, 0);
  atomic_init(&queue->bot, 0);
  queue->length = 0;
}

static inline void LCM_aqueue_push(LCM_aqueue_t* queue, void* val)
{
  // reserve a slot to write
  uint_fast64_t current_top =
      atomic_fetch_add_explicit(&queue->top, 1, LCIU_memory_order_relaxed);
  // write to the slot
  struct LCM_aqueue_entry_t* slot_p =
      &queue->container[current_top % queue->length];
  LCM_Assert(
      atomic_load_explicit(&slot_p->data, LCIU_memory_order_acquire) == 0,
      "wrote to a nonempty value!\n");
  atomic_store_explicit(&slot_p->data, (uint_least64_t)val,
                        LCIU_memory_order_release);
}

static inline void* LCM_aqueue_pop(LCM_aqueue_t* queue)
{
  uint_fast64_t current_bot =
      atomic_load_explicit(&queue->bot, LCIU_memory_order_relaxed);
  struct LCM_aqueue_entry_t* slot_p =
      &queue->container[current_bot % queue->length];
  uint_least64_t data =
      atomic_load_explicit(&slot_p->data, LCIU_memory_order_acquire);
  if (data == 0) {
    // the queue is empty
    LCII_PCOUNTERS_WRAPPER(
        LCII_pcounters[LCIU_get_thread_id()].lci_cq_pop_failed_empty++);
    return NULL;
  }
  //  LCM_DBG_Assert(current_top2 > current_bot, "bot %lu is ahead of top2
  //  %lu!\n", current_bot, current_top2);
  uint_fast64_t expected = current_bot;
  _Bool succeed = atomic_compare_exchange_strong_explicit(
      &queue->bot, &expected, current_bot + 1, LCIU_memory_order_relaxed,
      LCIU_memory_order_relaxed);
  if (!succeed) {
    // other thread is ahead of us
    LCII_PCOUNTERS_WRAPPER(
        LCII_pcounters[LCIU_get_thread_id()].lci_cq_pop_failed_contention++);
    return NULL;
  }
  // we have successfully reserve an entry
  // we can update the valid flag to tell the
  // producers they can safely write to this entry.
  atomic_store_explicit(&slot_p->data, 0, LCIU_memory_order_release);
#ifdef LCI_USE_PERFORMANCE_COUNTER
  uint_fast64_t current_top =
      atomic_load_explicit(&queue->top, LCIU_memory_order_relaxed);
  LCII_pcounters[LCIU_get_thread_id()].lci_cq_pop_len_accumulated +=
      current_top - current_bot;
  LCII_pcounters[LCIU_get_thread_id()].lci_cq_pop_succeeded++;
#endif
  return (void*)data;
}

#endif  // LCI_LCM_AQUEUE_H
