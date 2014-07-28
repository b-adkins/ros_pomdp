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
