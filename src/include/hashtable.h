#ifndef ARR_HASHTBL_H_
#define ARR_HASHTBL_H_

#include <stdint.h>
#include <stdlib.h>

typedef uintptr_t lc_value;
typedef uint64_t lc_key;

#include <assert.h>
#include <stdlib.h>

#include "macro.h"
#include "lock.h"

#define EMPTY ((uint64_t)-1)
#define TBL_BIT_SIZE 16
#define TBL_WIDTH 4

#ifdef __cplusplus
extern "C" {
#endif

struct lc_hash {
  union {
    struct {
      lc_key tag;
      lc_value val;
    } entry;
    struct {
      volatile int lock;
      struct lc_hash* next;
    } control;
  };
  // NOTE: This must be aligned to 16, make sure TBL_WDITH is 4,
  // So they will fit in a cache line.
};

enum insert_type { CLIENT, SERVER };
void lc_hash_create(struct lc_hash** h);
void lc_hash_destroy(struct lc_hash* h);

static inline int lc_hash_insert(struct lc_hash* h, lc_key key, lc_value* value,
                                 enum insert_type type);

#ifdef __cplusplus
}
#endif

// default values recommended by http://isthe.com/chongo/tech/comp/fnv/
static const uint32_t Prime = 0x01000193;  //   16777619
static const uint32_t Seed = 0x811C9DC5;   // 2166136261
#define TINY_MASK(x) (((uint32_t)1 << (x)) - 1)
#define FNV1_32_INIT ((uint32_t)2166136261)

static inline uint32_t myhash(const uint64_t k)
{
  uint32_t hash = ((k & 0xff) ^ Seed) * Prime;
  hash = (((k >> 8) & 0xff) ^ hash) * Prime;
  hash = (((k >> 16) & 0xff) ^ hash) * Prime;
  hash = (((k >> 24) & 0xff) ^ hash) * Prime;
  hash = (((k >> 32) & 0xff) ^ hash) * Prime;
  hash = (((k >> 40) & 0xff) ^ hash) * Prime;
  hash = (((k >> 48) & 0xff) ^ hash) * Prime;
  hash = (((k >> 56) & 0xff) ^ hash) * Prime;

  // Mask into smaller space.
  return (((hash >> TBL_BIT_SIZE) ^ hash) & TINY_MASK(TBL_BIT_SIZE));
}

static inline struct lc_hash* create_table(size_t num_rows)
{
  struct lc_hash* ret = 0;
  posix_memalign((void**)&ret, 64,
                 num_rows * TBL_WIDTH * sizeof(struct lc_hash));

  // Initialize all with EMPTY and clear lock.
  for (size_t i = 0; i < num_rows; i++) {
    // First are control.
    ret[i * TBL_WIDTH].control.lock = LC_SPIN_UNLOCKED;
    ret[i * TBL_WIDTH].control.next = NULL;

    // Remaining are slots.
    for (int j = 1; j < TBL_WIDTH; j++) {
      ret[i * TBL_WIDTH + j].entry.tag = EMPTY;
      ret[i * TBL_WIDTH + j].entry.val = 0;
    }
  }
  return ret;
}

static inline void free_table(struct lc_hash* p)
{
  free(p);
}

static inline int lc_hash_insert(struct lc_hash* h, lc_key key, lc_value* value,
                                 enum insert_type type)
{
  struct lc_hash* tbl_ = (struct lc_hash*)h;

  const uint32_t hash = myhash(key);
  const int bucket = hash * TBL_WIDTH;
  int checked_slot = 0;

  struct lc_hash* master = &tbl_[bucket];
  struct lc_hash* hcontrol = &tbl_[bucket];
  struct lc_hash* hentry = hcontrol + 1;
  struct lc_hash* empty_hentry = NULL;

  lc_key cmp_key = (key << 1) | (1 - type);

  lc_spin_lock(&master->control.lock);
  while (1) {
    lc_key tag = hentry->entry.tag;
    // If the key is the same as tag, someone has inserted it.
    // If the type is different, meaning we can't use it.
    if (tag == cmp_key) {
      *value = hentry->entry.val;
      hentry->entry.tag = EMPTY;
      lc_spin_unlock(&master->control.lock);
      return 0;
    } else if (tag == EMPTY) {
      // Ortherwise, if the tag is empty, we record the slot.
      // We can't return until we go over all entries.
      if (empty_hentry == NULL) empty_hentry = hentry;
    } else {
      // If we are still seeing some non-empty,
      // push that empty entry even further.
      empty_hentry = NULL;
    }

    hentry++;
    checked_slot++;
    // If we go over all entry, means no empty slot.
    if (checked_slot == (TBL_WIDTH - 1)) {
      // Moving on to the next.
      // *** SLOWISH ***
      if (hcontrol->control.next == NULL) {
        // This is the end of the table, if we still not found
        // create new table.
        if (empty_hentry == NULL) {
          hcontrol->control.next = create_table(1);
          hcontrol = hcontrol->control.next;
          empty_hentry = hcontrol + 1;
        }
        break;
      } else {
        // Otherwise, moving on.
        hcontrol = hcontrol->control.next;
        hentry = hcontrol + 1;
        checked_slot = 0;
      }
    }
  }
  empty_hentry->entry.tag = (key << 1) | type;
  empty_hentry->entry.val = *value;
  lc_spin_unlock(&master->control.lock);
  return 1;
}

#define LCI_MT_s lc_hash

#endif
