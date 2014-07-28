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

