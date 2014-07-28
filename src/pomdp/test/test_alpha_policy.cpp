/**
 * @file
 *
 * Tests for pomdp::AlphaPolicy.
 *
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

#include <gtest/gtest.h>

#include "alpha_policy.h"

using namespace pomdp;

TEST(AlphaPolicyTestLoad, testLoadPolicyInvalidPath)
{
  AlphaPolicy p_stack;
  EXPECT_FALSE(p_stack.loadFromFile("invalid path to nothing"));

  AlphaPolicy* p_heap = new AlphaPolicy("invalid path to nothing");
  EXPECT_FALSE(p_heap->isInit());
  delete p_heap;
}



int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
