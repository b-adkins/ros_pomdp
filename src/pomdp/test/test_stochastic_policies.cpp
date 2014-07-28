/**
 * @file
 *
 * Tests for stochastic policies. Currently, just pomdp::MultinomialPolicy.
 *
 * @date Aug 19, 2013
 * @author Bea Adkins
 */

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <string.h>

#include "multinomial_policy.h"
#include "simple_feature_space.h"

using namespace pomdp;
namespace bfs = boost::filesystem;

const bfs::path test_root("../../src/pomdp/test/");

TEST(MultinomialPolicy, testSample)
{
  // Makes it possible to test a protected method.
  class MultinomialMDPPolicyExposer : public MultinomialMDPPolicy
  {
  public:
    int sample() const{ return MultinomialMDPPolicy::sample(); }
  };

  MultinomialMDPPolicyExposer p;
  const int n_models = 5;

  std::vector<std::string> paths(n_models, bfs::path(test_root/"dummy.mdp_policy").string()); //Empty models
  p.setStateSpace(boost::make_shared<SimpleFeatureSpace>(2));
  p.setActionSpace(boost::make_shared<SimpleFeatureSpace>(2));
  ASSERT_TRUE(p.loadFromFiles(paths));

  const double model_likelihoods_c[n_models] = { 0.01, 0.09, 0.2, 0.3, 0.4};
  std::vector<double> model_likelihoods(model_likelihoods_c, model_likelihoods_c + n_models);
  ASSERT_TRUE(p.setModelLikelihoods(model_likelihoods));

  std::vector<unsigned int> sample_counts(5, 0);
  const unsigned int n_samples = 10000000;
  for(int i = 0; i < n_samples; i++)
  {
    int sampled_model_index = p.sample();
//    ASSERT_TRUE(sampled_model_index != -1); // Uncomment if range assert removed.
    ASSERT_TRUE(0 <= sampled_model_index && sampled_model_index < n_models);
    sample_counts[sampled_model_index]++;
  }

  // Check that sampled data matches the supplied probabilities.
  int i = 5;
  for(int i = 0; i < n_models; i++)
    ASSERT_NEAR(model_likelihoods_c[i], double(sample_counts[i])/double(n_samples), 0.001);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
