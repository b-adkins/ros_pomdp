/**
 *
 * Tests for pomdp::MDPPolicy.
 *
 *
 * @date Jul 16, 2013
 * @author Bea Adkins
 */

#include <gtest/gtest.h>

#include "map_policy.h"
#include "simple_feature_space.h"
#include "simple_feature_value.h"

using namespace pomdp;

const char* path_invalid = "invalid path to nothing";

TEST(TestMDPPolicyLoad, testLoadPolicyInvalidPath)
{
  MapPolicy p1;
  EXPECT_FALSE(p1.loadFromFile(path_invalid));
//  EXPECT_FALSE(p_stack.loadFromFile(std::string(path_invalid)));

  MapPolicy p2(path_invalid);
  EXPECT_FALSE(p2.isInit());

//  p2 = new MDPPolicy(std::string(path_invalid));
//  EXPECT_FALSE(p_heap->isInit());
//  delete p_heap;
}

const char* path_4x3 = "../../src/pomdp/test/4x3.95.bogus.mdp_policy";

class TestMDPPolicy : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    shared_ptr<const FeatureSpace> ss = boost::make_shared<const SimpleFeatureSpace>(11);
    shared_ptr<const FeatureSpace> as = boost::make_shared<const SimpleFeatureSpace>(11);
    mdp_4x3.setStateSpace(ss);
    ASSERT_TRUE(mdp_4x3.loadFromFile(path_4x3));
  }

  MapPolicy mdp_4x3;
};

TEST_F(TestMDPPolicy, testNumStates)
{
  EXPECT_EQ(11, mdp_4x3.getNumStates());
}

TEST_F(TestMDPPolicy, testApplyPolicyBounds)
{
  EXPECT_NE(-1, mdp_4x3.applyPolicy(SimpleFeatureValue(0)));
  EXPECT_NE(-1, mdp_4x3.applyPolicy(SimpleFeatureValue(10)));
  EXPECT_EQ(-1, mdp_4x3.applyPolicy(SimpleFeatureValue(11)));
}

TEST_F(TestMDPPolicy, testApplyPolicyLookup)
{
  EXPECT_EQ(3, mdp_4x3.applyPolicy(SimpleFeatureValue(2)));
  EXPECT_EQ(1, mdp_4x3.applyPolicy(SimpleFeatureValue(7)));
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
