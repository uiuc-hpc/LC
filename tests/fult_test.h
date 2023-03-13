#include "ult/fult/fult.h"

TEST(FULT, InitFini)
{
  fworker* f = nullptr;
  fworker_create(&f);
  ASSERT_NE(f, nullptr);
  fworker_start_main(f);
  fworker_stop_main(f);
  fworker_destroy(f);
}

static void* incr(void* arg)
{
  int* v = (int*)arg;
  (*v)++;
  return 0;
}

TEST(FULT, SpawnJoinMain)
{
  fworker* f;
  fworker_create(&f);
  fworker_start_main(f);
  int v = 0;
  auto* t = fworker_spawn(f, incr, &v, 4096);
  ASSERT_NE(t, nullptr);
  fthread_join(t);
  ASSERT_EQ(v, 1);
  fworker_stop_main(f);
  fworker_destroy(f);
}

TEST(FULT, SpawnJoinExt)
{
  fworker *f, *ext;
  fworker_create(&f);
  fworker_start_main(f);
  fworker_create(&ext);
  fworker_start(ext);
  int v = 0;
  auto* t = fworker_spawn(ext, incr, &v, 4096);
  ASSERT_NE(t, nullptr);
  fthread_join(t);
  ASSERT_EQ(v, 1);
  fworker_stop(ext);
  fworker_destroy(ext);
  fworker_stop_main(f);
  fworker_destroy(f);
}
