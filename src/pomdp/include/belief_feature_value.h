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
