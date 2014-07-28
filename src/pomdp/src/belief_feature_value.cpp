/**
 * @file
 *
 *
 *
 * @date Sep 7, 2013
 * @author Bea Adkins
 */

#include "belief_feature_value.h"

namespace pomdp
{

BeliefFeatureValue::BeliefFeatureValue()
{
}

BeliefFeatureValue::BeliefFeatureValue(const std::vector<double>& belief_state) : belief_state_(belief_state)
{
}

BeliefFeatureValue::BeliefFeatureValue(const BeliefFeatureValue& s) : belief_state_(s.belief_state_)
{
}

BeliefFeatureValue& BeliefFeatureValue::operator=(BeliefFeatureValue const& s)
{
  if(this != &s)
    belief_state_ = s.asVector();

  return *this;
}

BeliefFeatureValue& BeliefFeatureValue::operator=(std::vector<double> const& b)
{
  return operator=(BeliefFeatureValue(b));
}

} /* namespace pomdp */
