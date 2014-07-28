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
 * @file
 *
 *
 *
 * @date Sep 7, 2013
 * @author Bea Adkins
 */

#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <ros/console.h>
#include <ros/assert.h>
#include <vector>

#include "feature_value.h"

namespace pomdp
{

/**
 * Represents an uncertain, discrete FeatureValue as a vector of the probabilities that each discrete value is the true
 * value. E.g. a POMDP belief state.
 */
class BeliefFeatureValue : public FeatureValue
{
public:
  BeliefFeatureValue();
  /** Constructs a POMDPState. */
  BeliefFeatureValue(const std::vector<double>& belief_state);
  /** Copy constructor. */
  BeliefFeatureValue(const BeliefFeatureValue& s);
  virtual ~BeliefFeatureValue(){};

  // Written this way (v.s. using at()) to allow bounds checking to be compiled out.
  double const& operator[](unsigned int i) const
  {
    ROS_ASSERT(i < belief_state_.size());
    return belief_state_[i];
  }

  BeliefFeatureValue& operator=(BeliefFeatureValue const& s);
  BeliefFeatureValue& operator=(std::vector<double> const& b);

  /**
   * Use this struct as a double*.
   *
   * Does not check validity of data pointer! That's what isValid(int) is for!
   *
   * @return Belief state. Only valid while the referenced vector does not reallocate!!
   */
  const double* asArray() const{ return &belief_state_[0]; };

  /**
   * Use this struct as an STL vector.
   *
   * Does not check validity of data reference! That's what isValid(int) is for!
   *
   * @return Belief state.
   */
  const std::vector<double>& asVector() const{ return belief_state_; };

protected:
  std::vector<double> belief_state_;
};

} /* namespace pomdp */
#endif /* BELIEF_STATE_H */
