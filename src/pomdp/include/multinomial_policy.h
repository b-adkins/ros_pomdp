/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Boone "Bea" Adkins
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/**
 * @class pomdp::MultinomialPolicy
 *
 * CompoundPolicy whose action is determined by sampling a <s>multinomial</s> categorical distribution across
 * sub-policies.
 *
 * @date Aug 10, 2013
 * @author Bea Adkins
 */

#ifndef MULTINOMIAL_POLICY_H
#define MULTINOMIAL_POLICY_H

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include "assert_rng.h"
#include "compound_policy.h"

namespace pomdp
{
template <class Base_Policy>
class MultinomialPolicy : public CompoundPolicySpec<Base_Policy>
{
public:
  MultinomialPolicy() : val_table_(NULL)
  {
    rng_ = gsl_rng_alloc(gsl_rng_taus2);

    // Randomly seed the RNG.
    srand(time(NULL));
    gsl_rng_set (rng_, rand());
  }

  virtual ~MultinomialPolicy()
  {
    gsl_rng_free(rng_);
    if(CompoundPolicySpec<Base_Policy>::isInit())
    {
      gsl_ran_discrete_free(val_table_);
    }
  }

  virtual bool loadFromFiles(const std::vector<std::string>& paths)
  {
    if(!CompoundPolicySpec<Base_Policy>::loadFromFiles(paths))
      return false;
    CompoundPolicySpec<Base_Policy>::is_init_ = false; // Not yet initialized.


    // Now that model number is known, initialize random number variables.
    freshenDistribution();

    CompoundPolicySpec<Base_Policy>::is_init_ = true;
    return true;
  }

  virtual int applyPolicy(const FeatureValue& current) const
  {
    // Sample a policy index.
    int i_sampled_policy = sample();
    if(i_sampled_policy == -1)
      return -1;
    ROS_ASSERT(0 <= i_sampled_policy && i_sampled_policy < CompoundPolicySpec<Base_Policy>::getNumModels());

    // Apply that policy.
    ROS_DEBUG_STREAM("Appying policy #" << i_sampled_policy << ":");
    return CompoundPolicySpec<Base_Policy>::models_[i_sampled_policy]->applyPolicy(current);
  }

  virtual bool setModelLikelihoods(const std::vector<double>& model_likelihoods)
  {
    if(!CompoundPolicySpec<Base_Policy>::setModelLikelihoods(model_likelihoods))
      return false;

    freshenDistribution(); // With new model likelihoods
    return true;
  }
protected:
  /**
   * Random number generator.
   */
  gsl_rng* rng_;

  /**
   * Table for quickly sampling values.
   */
  gsl_ran_discrete_t* val_table_;

  /**
   * @return int within [0, numModels] or -1 on error.
   */
  int sample() const
  {
    if(!CompoundPolicySpec<Base_Policy>::isInit() || CompoundPolicySpec<Base_Policy>::getNumModels() == 0)
      return -1;

    // Sample a value.
    return gsl_ran_discrete(rng_, val_table_);
  }

  /**
   * Call this whenever model_likelihoods_ change, in value or length.
   */
  void freshenDistribution()
  {
    ROS_ASSERT(CompoundPolicySpec<Base_Policy>::model_likelihoods_.size() > 0);

    if(CompoundPolicySpec<Base_Policy>::isInit())
      gsl_ran_discrete_free(val_table_);
    val_table_ = gsl_ran_discrete_preproc(CompoundPolicySpec<Base_Policy>::model_likelihoods_.size(),
                                          &CompoundPolicySpec<Base_Policy>::model_likelihoods_[0]);
  }
};

typedef MultinomialPolicy<MDPPolicy> MultinomialMDPPolicy;
typedef MultinomialPolicy<POMDPPolicy> MultinomialPOMDPPolicy;

} /* namespace pomdp */
#endif /* MULTINOMIAL_POLICY_H */
