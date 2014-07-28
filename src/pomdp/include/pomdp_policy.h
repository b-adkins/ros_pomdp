/**
 * @class pomdp::POMDPPolicy
 *
 * Abstract base class for policies valid for POMDPs.
 *
 * Provides a useful macro to check state validity and an "isa" relationship for compile-time/initialization-time
 * checking of Policies.
 *
 * @date Aug 9, 2013
 * @author Bea Adkins
 */

#ifndef POMDP_POLICY_H
#define POMDP_POLICY_H

#include <ros/assert.h>

#include "policy.h"

namespace pomdp
{

class POMDPPolicy : public virtual Policy
{
public:
  POMDPPolicy(){};
  virtual ~POMDPPolicy(){};
};

} /* namespace pomdp */
#endif /* POMDP_POLICY_H */
