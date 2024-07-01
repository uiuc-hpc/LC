#ifndef LCI_PERFORMANCE_COUNTER_H
#define LCI_PERFORMANCE_COUNTER_H

extern LCT_pcounter_ctx_t LCII_pcounter_ctx;

// clang-format off
#define LCII_PCOUNTER_NONE_FOR_EACH(_macro)  \
    _macro(get_packet_timer)                 \
    _macro(get_packet_pool_id_timer)         \
    _macro(get_packet_lock_timer)            \
    _macro(get_packet_local_timer)           \
    _macro(get_packet_unlock_timer)          \
    _macro(cq_push_internal)                 \
    _macro(cq_push_faa)                      \
    _macro(cq_push_write)                    \
    _macro(cq_push_store)                    \

#define LCII_PCOUNTER_TREND_FOR_EACH(_macro) \
    _macro(send)                             \
    _macro(put)                              \
    _macro(recv)                             \
    _macro(comp_produce)                     \
    _macro(comp_consume)                     \
    _macro(net_sends_posted)                 \
    _macro(net_send_posted)                  \
    _macro(net_recv_posted)                  \
    _macro(net_recv_failed_lock)             \
    _macro(net_send_comp)                    \
    _macro(net_recv_comp)                    \
    _macro(net_send_failed_lock)             \
    _macro(net_send_failed_nomem)            \
    _macro(net_recv_failed_nopacket)         \
    _macro(net_poll_cq_attempts)             \
    _macro(net_poll_cq_calls)                \
    _macro(net_poll_cq_entry_count)          \
    _macro(progress_call)                    \
    _macro(packet_get)                       \
    _macro(packet_put)                       \
    _macro(packet_stealing)                  \
    _macro(packet_stealing_succeeded)        \
    _macro(packet_stealing_failed)           \
    _macro(backlog_queue_push)               \
    _macro(backlog_queue_pop)                \
    _macro(expected_msg)                     \
    _macro(unexpected_msg)

#define LCII_PCOUNTER_TIMER_FOR_EACH(_macro) \
    _macro(sync_stay_timer)                  \
    _macro(cq_stay_timer)                    \
    _macro(useful_progress_timer)            \
    _macro(refill_rq_timer)                  \
    _macro(update_posted_recv)               \
    _macro(post_recv_timer)                  \
    _macro(get_recv_packet_timer)            \
    _macro(cq_push_timer)                    \
    _macro(cq_pop_timer)                     \
    _macro(serve_rts_timer)                  \
    _macro(rts_mem_timer)                    \
    _macro(rts_send_timer)                   \
    _macro(serve_rtr_timer)                  \
    _macro(rtr_mem_reg_timer)                \
    _macro(rtr_put_timer)                    \
    _macro(serve_rdma_timer)                 \
    _macro(packet_stealing_timer)            \
    _macro(mem_reg_timer)                    \
    _macro(mem_dereg_timer)                  \
    _macro(net_mem_reg_timer)                \
    _macro(net_mem_dereg_timer)
// clang-format on

#define LCII_PCOUNTER_HANDLE_DECL(name) \
  extern LCT_pcounter_handle_t LCII_pcounter_handle_##name;

LCII_PCOUNTER_NONE_FOR_EACH(LCII_PCOUNTER_HANDLE_DECL)
LCII_PCOUNTER_TREND_FOR_EACH(LCII_PCOUNTER_HANDLE_DECL)
LCII_PCOUNTER_TIMER_FOR_EACH(LCII_PCOUNTER_HANDLE_DECL)

#ifdef LCI_USE_PERFORMANCE_COUNTER
#define LCII_PCOUNTER_ADD(name, val) \
  LCT_pcounter_add(LCII_pcounter_ctx, LCII_pcounter_handle_##name, val);
#define LCII_PCOUNTER_START(name) \
  LCT_pcounter_start(LCII_pcounter_ctx, LCII_pcounter_handle_##name);
#define LCII_PCOUNTER_END(name) \
  LCT_pcounter_end(LCII_pcounter_ctx, LCII_pcounter_handle_##name);
#define LCII_PCOUNTER_NOW(time) LCT_time_t time = LCT_now();
#define LCII_PCOUNTER_STARTT(name, time) \
  LCT_pcounter_startt(LCII_pcounter_ctx, LCII_pcounter_handle_##name, time);
#define LCII_PCOUNTER_ENDT(name, time) \
  LCT_pcounter_endt(LCII_pcounter_ctx, LCII_pcounter_handle_##name, time);
#else
#define LCII_PCOUNTER_ADD(name, val)
#define LCII_PCOUNTER_START(name)
#define LCII_PCOUNTER_END(name)
#define LCII_PCOUNTER_NOW(time)
#define LCII_PCOUNTER_STARTT(name, time)
#define LCII_PCOUNTER_ENDT(name, time)
#endif

void LCII_pcounters_init();
void LCII_pcounters_fina();

#endif  // LCI_PERFORMANCE_COUNTER_H
