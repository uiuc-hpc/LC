/*
 * Copyright (C) 1999-2001 The Regents of the University of California
 * (through E.O. Lawrence Berkeley National Laboratory), subject to
 * approval by the U.S. Department of Energy.
 *
 * Use of this software is under license. The license agreement is included
 * in the file MVICH_LICENSE.TXT.
 *
 * Developed at Berkeley Lab as part of MVICH.
 *
 * Authors: Bill Saphir      <wcsaphir@lbl.gov>
 *          Michael Welcome  <mlwelcome@lbl.gov>
 */

/* Copyright (c) 2001-2016, The Ohio State University. All rights
 * reserved.
 *
 * This file is part of the MVAPICH2 software package developed by the
 * team members of The Ohio State University's Network-Based Computing
 * Laboratory (NBCL), headed by Professor Dhabaleswar K. (DK) Panda.
 *
 * For detailed copyright and licensing information, please refer to the
 * copyright file COPYRIGHT in the top level MVAPICH2 directory.
 *
 */

/*
 *
 * dreg.h
 *
 * Interface for dynamic registration of memory.
 */

#ifndef _DREG_H
#define _DREG_H

#include <stdint.h>
#include <stdlib.h>

#if !defined(DISABLE_PTMALLOC)
int have_dereg(void);
void lock_dereg(void);
void unlock_dereg(void);
int have_dreg(void);
void lock_dreg(void);
void unlock_dreg(void);
#endif

#define MAX_NUM_HCAS 1

extern uintptr_t get_dma_mem(void* server, void* buf, size_t s);
extern int free_dma_mem(uintptr_t mem);


typedef struct dreg_entry dreg_entry;

extern int g_is_dreg_initialized;
extern int g_is_dreg_finalize;
extern unsigned long dreg_stat_cache_hit;
extern unsigned long dreg_stat_cache_miss;
struct dreg_entry {
    unsigned long pagenum;
    uintptr_t memhandle[MAX_NUM_HCAS];

    int refcount;

    /* page number, e.g. address >> DREG_PAGEBITS */
    unsigned long  npages;

    /* for hash chain or free chain */
    dreg_entry *next;

    /* for zero refcount chain */
    dreg_entry *next_unused;
    dreg_entry *prev_unused;

    void* server;
    int is_valid;
};


/*
 * When an application needs to register memory, it
 * calls dreg_register. The application does not keep
 * track of what has already been registered. This is
 * tracked inside the dreg interface.
 *
 * dreg stores dreg entries in a hash table for
 * easy lookup. The table is indexed by a dreg
 * page number (the dreg page size may be unrelated
 * to the machine page size, but is a power of two).
 *
 * The hash table size is DREG_HASHSIZE. Collisions
 * are handled by chaining through the "next" field
 * in dreg_entry. New entries are placed at the head
 * of the chain.
 *
 * Registrations are reference-counted. An application
 * is responsible for pairing dreg_register/unregister
 * calls.
 *
 * When the reference count reaches zero, the dreg_entry is moved to
 * the end of the hash chain (xxx this is NOT clearly the right
 * thing. For now, leave it where it is and see how it works) and also
 * placed on the unused_list (it is now on two lists instead of
 * one). Associated memory * is *not* unregistered with VIA,
 * but is a candidate for VIA unregistration if needed. The
 * unused list is a doubly linked list so that entries can be removed
 * from the middle of the list if an application registration request
 * comes along before memory is actually unregistered. Also, the list
 * is FIFO rather than LIFO (e.g. it has both a tail and a head) so
 * that only entries that have been on the list a long time become
 * candidates for VIA deregistration.
 *
 * In summary, there are three lists.
 *  - the dreg free list. pool of dreg structures from which
 *    we grab new dreg entries as needed. LIFO (head only) for
 *    cache reuse. Singly linked through next field in dreg_entry
 *
 *  - hash list chain. When there is a hash collision, entries
 *    with the same hash are placed on this list. It is singly
 *    linked through the next field in dreg_entry. New entries
 *    are placed at the head of the list. Because the list is
 *    singly linked, removing an entry from the middle is potentially
 *    expensive. However, hash lists should be short, and the only
 *    time we remove an entry is if it has zero ref count and we
 *    have to unregister its memory.
 *
 *  - unused list. This is the list of dreg entries that represent
 *    registered memory, but the application is not currently using
 *    this memory. Rather than deregister memory when the refcount
 *    goes to zero, we put it on the unused list. If resources
 *    become scarce and we have to unregister memory, it is easy to
 *    find entries with zero ref count.
 *    NOTE adding/removing entries to/from unused list is a critical
 *    path item that will happen all the time. Also,
 *    the need to find a unused item is very rare, and is associated
 *    with a VIA deregistration/registration. So why do we want
 *    the unused list? It is an LRU device that ensures that only
 *    memory that has not been used for a while will be freed.
 *    This avoids a serious thrashing scenario.
 *    xxx consider deleting/replacing unused list later on.
 */

/* LAZY_MEM_UNREGISTER, enabled, will not un-register memory
 * after the ref-count drops to zero, rather the entry will
 * be put on the unused list in case the memory is to be
 * used again.  These are the semantics described above.
 * This can be a problem if the virtual to physical mapping
 * gets changed between calls.
 * By undefining this macro, we revert to semantics in which
 * memory is unregistered when the ref_count drops to zero.
 * In this case, the unused list should always be empty.
 *
 * NOTE: If not doing RDMA operations, we (MVICH layer)
 * controls all registered memory (VBUFs) and we don't
 * have to worry about the address translation getting
 * changed.
 */

extern struct dreg_entry *dreg_free_list;

extern struct dreg_entry *dreg_unused_list;
extern struct dreg_entry *dreg_unused_tail;



/* DREG_PAGESIZE must be smaller than or equal to the hardware
 * pagesize. Otherwise we might register past the top page given
 * to us. This page might be invalid (e.g. read-only).
 */

#define DREG_PAGESIZE 4096      /* must be 2^DREG_PAGEBITS */
#define DREG_PAGEBITS 12        /* must be ln2 of DREG_PAGESIZE */
#define DREG_PAGEMASK (DREG_PAGESIZE - 1)

#define DREG_HASHSIZE 128       /* must be 2^DREG_HASHBITS */
#define DREG_HASHBITS 7         /* must be ln2 DREG_HASHSIZE */
#define DREG_HASHMASK (DREG_HASHSIZE-1)

#define DREG_HASH(a) ( ( ((uintptr_t)(a)) >> DREG_PAGEBITS) & DREG_HASHMASK )

/*
 * Delete entry d from the double-linked unused list.
 * Take care if d is either the head or the tail of the list.
 */

#define DREG_REMOVE_FROM_UNUSED_LIST(d) {                           \
    dreg_entry *prev = (d)->prev_unused;                            \
        dreg_entry *next = (d)->next_unused;                        \
        (d)->next_unused = NULL;                                    \
        (d)->prev_unused = NULL;                                    \
        if (prev != NULL) {                                         \
            prev->next_unused = next;                               \
        }                                                           \
    if (next != NULL) {                                             \
        next->prev_unused = prev;                                   \
    }                                                               \
    if (dreg_unused_list == (d)) {                                  \
        dreg_unused_list = next;                                    \
    }                                                               \
    if (dreg_unused_tail == (d)) {                                  \
        dreg_unused_tail = prev;                                    \
    }                                                               \
}

/*
 * Add entries to the head of the unused list. dreg_evict() takes
 * them from the tail. This gives us a simple LRU mechanism
 */

#define DREG_ADD_TO_UNUSED_LIST(d) {                                \
    d->next_unused = dreg_unused_list;                              \
    d->prev_unused = NULL;                                          \
    if (dreg_unused_list != NULL) {                                 \
        dreg_unused_list->prev_unused = d;                          \
    }                                                               \
    dreg_unused_list = d;                                           \
    if (NULL == dreg_unused_tail) {                                 \
        dreg_unused_tail = d;                                       \
    }                                                               \
}

#define DREG_GET_FROM_FREE_LIST(d) {                                \
    d = dreg_free_list;                                             \
    if (dreg_free_list != NULL) {                                   \
        dreg_free_list = dreg_free_list->next;                      \
        dreg_inuse_count++;                                         \
        if (dreg_max_use_count < dreg_inuse_count)                  \
            dreg_max_use_count = dreg_inuse_count;                  \
    }                                                               \
}

#define DREG_ADD_TO_FREE_LIST(d) {                                  \
    d->next = dreg_free_list;                                       \
    dreg_free_list = d;                                             \
    dreg_inuse_count--;                                             \
}

int dreg_init(void);

int dreg_finalize(void);

dreg_entry *dreg_register(void* server, void *buf, size_t len);

void dreg_unregister(dreg_entry * entry);

dreg_entry *dreg_find(void *buf, size_t len);

dreg_entry *dreg_get(void);

int dreg_evict(void);

void dreg_release(dreg_entry * d);

void dreg_decr_refcount(dreg_entry * d);

void dreg_incr_refcount(dreg_entry * d);

dreg_entry *dreg_new_entry(void* server, void *buf, size_t len);

void flush_dereg_mrs_external(void);

#ifndef DISABLE_PTMALLOC
void find_and_free_dregs_inside(void *buf, size_t len);
#endif

#if defined(CKPT) ||  defined(ENABLE_CHECKPOINTING)
void dreg_deregister_all(void);
void dreg_reregister_all(void);
#endif

#endif                          /* _DREG_H */
