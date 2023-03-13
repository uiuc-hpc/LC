#include "hashtable.h"

TEST(HASH, InitFini)
{
  lc_hash* h = create_table(1 << TBL_BIT_SIZE);
  ASSERT_NE(h, nullptr);
  free_table(h);
}

TEST(HASH, InsertServerFirst)
{
  lc_hash* h = create_table(1 << TBL_BIT_SIZE);
  lc_value v = 1;
  lc_key k = 1;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, SERVER));
  ASSERT_EQ(v, 1);
  v = 2;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, SERVER));
  ASSERT_EQ(v, 2);
  v = -3;
  ASSERT_FALSE(lc_hash_insert(h, k, &v, CLIENT));
  ASSERT_TRUE(v == 1 || v == 2);
  v = -3;
  ASSERT_FALSE(lc_hash_insert(h, k, &v, CLIENT));
  ASSERT_TRUE(v == 1 || v == 2);
  v = -3;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, CLIENT));
  ASSERT_EQ(v, -3);
  free_table(h);
}

TEST(HASH, InsertClientFirst)
{
  lc_hash* h = create_table(1 << TBL_BIT_SIZE);
  lc_value v = 1;
  lc_key k = 1;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, CLIENT));
  ASSERT_EQ(v, 1);
  v = 2;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, CLIENT));
  ASSERT_EQ(v, 2);
  v = -3;
  ASSERT_FALSE(lc_hash_insert(h, k, &v, SERVER));
  ASSERT_TRUE(v == 1 || v == 2);
  v = -3;
  ASSERT_FALSE(lc_hash_insert(h, k, &v, SERVER));
  ASSERT_TRUE(v == 1 || v == 2);
  v = -3;
  ASSERT_TRUE(lc_hash_insert(h, k, &v, SERVER));
  ASSERT_EQ(v, -3);
  free_table(h);
}
