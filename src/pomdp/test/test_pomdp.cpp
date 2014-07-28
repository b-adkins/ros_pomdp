#include <gtest/gtest.h>

#include "pomdp.h"

using namespace pomdp;


TEST(POMDPTest, testLoadMDPInvalidPath)
{
  POMDP p;
  EXPECT_FALSE(p.loadFromFile("invalid path to nothing"));
}



int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
