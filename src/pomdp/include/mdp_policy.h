/**
 * @class pomdp::MDPPolicy
 *
 * Abstract base class for policies valid for POMDPs.
 *
 * Provides a useful macro to check state validity and an "isa" relationship for compile-time/initialization-time
 * checking of Policies.
 *
 * @date Aug 9, 2013
 * @author Bea Adkins
 */

#ifndef MDP_POLICY_H
#define MDP_POLICY_H

#include "policy.h"

namespace pomdp
{

class MDPPolicy : public virtual Policy
{
public:
  MDPPolicy(){};
  virtual ~MDPPolicy(){};
};

} /* namespace pomdp */
#endif /* MDP_POLICY_H */
