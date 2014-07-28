/**
 * @file
 *
 *
 *
 * @date Jul 15, 2013
 * @author Bea Adkins
 */

#include "policy.h"
#include "feature_space.h"

namespace pomdp
{

/**
 * Finds a Policy within a <s>container</s> vector (for now) of Policy objects
 * by name field.
 *
 * @param policies List of policies to search.
 * @param name Name of desired policy object.
 * @return Smart pointer to first Policy object found matching 'name'. Empty (boolean value of false) if not found.
 */
shared_ptr<Policy> Policy::findPolicy(const std::vector<shared_ptr<Policy> >& policies, std::string name)
{
  for(int i = 0; i < policies.size(); i++)
  {
    if(policies[i]->getName() == name)
      return policies[i];
  }

  return shared_ptr<Policy>();
}

unsigned int Policy::getNumStates() const
{
  if(!state_space_)
    return 0;
  return state_space_->getNumValues();
}

} /* namespace pomdp */

