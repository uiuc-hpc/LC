#include <gtest/gtest.h>

#include "hash_test.h"
#include "pool_test.h"
//#include "fult_test.h"

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
