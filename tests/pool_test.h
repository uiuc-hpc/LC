#include "pool.h"

TEST(POOL, InitFini)
{
  lc_pool* h = nullptr;
  lc_pool_create(&h);
  ASSERT_NE(h, nullptr);
  lc_pool_destroy(h);
}

TEST(POOL, Put)
{
  lc_pool* h;
  lc_pool_create(&h);
  lc_pool_put(h, (void*)1);
  lc_pool_put(h, (void*)2);
  ASSERT_EQ(lc_pool_get(h), (void*)2);
  ASSERT_EQ(lc_pool_get(h), (void*)1);
  lc_pool_destroy(h);
}

TEST(POOL, PutTo)
{
  lc_pool* h;
  lc_pool_create(&h);
  lc_pool_put(h, (void*)1);
  lc_pool_put_to(h, (void*)2, 0);
  ASSERT_EQ(lc_pool_get(h), (void*)2);
  ASSERT_EQ(lc_pool_get(h), (void*)1);
  lc_pool_destroy(h);
}

TEST(POOL, GetNB)
{
  lc_pool* h;
  lc_pool_create(&h);
  lc_pool_put(h, (void*)1);
  ASSERT_EQ(lc_pool_get(h), (void*)1);
  ASSERT_EQ(lc_pool_get_nb(h), (void*)POOL_EMPTY);
  lc_pool_put(h, (void*)1);
  ASSERT_EQ(lc_pool_get_nb(h), (void*)1);
  lc_pool_destroy(h);
}
