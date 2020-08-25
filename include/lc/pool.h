#ifndef LC_POOL_H_
#define LC_POOL_H_

#include <pthread.h>
#include <sched.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "thread.h"
#include "lc/lock.h"
#include "lc/dequeue.h"

#define MAX_NPOOLS 272
#define MAX_LOCAL_POOL 32 // align to a cache line.

LC_EXPORT extern int32_t tls_pool_struct[MAX_NPOOLS][MAX_LOCAL_POOL];
LC_EXPORT extern volatile int init_lock;

struct dequeue;

#ifdef __cplusplus
extern "C" {
#endif

LC_INLINE int lc_worker_id(void)
{
  if (unlikely(lcg_core_id == -1)) {
    lcg_core_id = sched_getcpu();
    if (lcg_core_id == -1) {
      lcg_core_id = __sync_fetch_and_add(&lcg_current_id, 1);
    }
  }
  return lcg_core_id;
}

typedef struct lc_pool {
  int key;
  int32_t npools;
  struct dequeue* lpools[MAX_NPOOLS];
} lc_pool __attribute__((aligned(64)));

LC_EXPORT
void lc_pool_create(lc_pool** pool);

LC_EXPORT
void lc_pool_destroy(lc_pool* pool);

LC_INLINE
void lc_pool_put(lc_pool* pool, void* elm);

LC_INLINE
void lc_pool_put_to(lc_pool* pool, void* elm, int32_t pid);

LC_INLINE
void* lc_pool_get(lc_pool* pool);

LC_INLINE
void* lc_pool_get_nb(lc_pool* pool);

#define POOL_EMPTY (NULL)

#ifdef __cplusplus
}
#endif

#define POOL_UNINIT ((int32_t)-1)

LC_INLINE int32_t lc_pool_get_local_id(struct lc_pool* pool)
{
  int wid = lc_worker_id();
  int32_t pid = tls_pool_struct[wid][pool->key];
  if (unlikely(pid == POOL_UNINIT)) {
    lc_spin_lock(&init_lock);
    pid = tls_pool_struct[wid][pool->key];
    if (pid == POOL_UNINIT) {
      struct dequeue* lpool = 0;
      posix_memalign((void**) &lpool, 64, sizeof(struct dequeue));
      dq_init(lpool);
      pid = pool->npools++;
      pool->lpools[pid] = lpool;
      tls_pool_struct[wid][pool->key] = pid;
    }
    lc_spin_unlock(&init_lock);
  }
  // assert(pid >= 0 && pid < pool->npools && "POOL ERROR: pid out-of-range");
  return pid;
}

LC_INLINE int32_t lc_pool_get_steal_id(int32_t npools, int32_t pid)
{
  if (npools == 1)
    return -1; /* if only one pool, no one else to steal from */
  int32_t r = rand() % (npools - 1);
  return (r + pid + 1) % npools;
}

LC_INLINE void* lc_pool_steal_from(struct lc_pool* pool, int32_t pid)
{
  void* elm = NULL;
  if (likely(pool->lpools[pid] != NULL))
    elm = dq_pop_bot(pool->lpools[pid]);
  return elm;
}

LC_INLINE void* lc_pool_steal(struct lc_pool* pool, int32_t pid)
{
  void* elm = NULL;
  int32_t target = lc_pool_get_steal_id(pool->npools, pid);
  if (target != -1)
    elm = lc_pool_steal_from(pool, target);
  return elm;
}

LC_INLINE void lc_pool_put_to(struct lc_pool* pool, void* elm, int32_t pid)
{
  struct dequeue* lpool = pool->lpools[pid];
  dq_push_top(lpool, elm);
}

LC_INLINE void lc_pool_put(struct lc_pool* pool, void* elm)
{
  int32_t pid = lc_pool_get_local_id(pool);
  lc_pool_put_to(pool, elm, pid);
}

LC_INLINE void* lc_pool_get_nb(struct lc_pool* pool)
{
  int32_t pid = lc_pool_get_local_id(pool);
  struct dequeue* lpool = pool->lpools[pid];
  void* elm = dq_pop_top(lpool);
  if (elm == NULL)
    elm = lc_pool_steal(pool, pid);
  return elm;
}

LC_INLINE void* lc_pool_get(struct lc_pool* pool)
{
  int32_t pid = lc_pool_get_local_id(pool);
  struct dequeue* lpool = pool->lpools[pid];
  void* elm = NULL;
  do {
    /* must try self every iteration since we never steal from self */
    elm = dq_pop_top(lpool);
    if (elm == NULL)
      elm = lc_pool_steal(pool, pid);
  } while (elm == NULL);
  return elm;
}

#endif
