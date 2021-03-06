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
 * @class pomdp::Policy
 *
 * Base class for all policies. Allows one POMDP file to be easily used and
 * compile-time checking of policy type (in addition to the usual code reuse).
 *
 * @date Jul 15, 2013
 * @author Bea Adkins
 */

#ifndef POLICY_H
#define POLICY_H

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include "feature_value.h"

using boost::shared_ptr; // Because it's long and ugly enough v.s. a '*'.

namespace pomdp
{
// No need to tangle dependencies.
class Pretty;
class FeatureSpace;

class Policy
{
public:
  Policy() : is_init_(false), action_space_(), state_space_() {};
  virtual bool loadFromFile(const char* path) = 0;
  bool loadFromFile(const std::string& path){ return loadFromFile(path.c_str()); };
  virtual ~Policy(){};

  static shared_ptr<Policy> findPolicy(const std::vector<shared_ptr<Policy> >& policies, std::string name);

  virtual int applyPolicy(const FeatureValue& state) const = 0;

  unsigned int getNumStates() const;
  bool isInit() const{ return is_init_; };
  std::string getName() const { return name_; }
  void setName(std::string name){ this->name_ = name; }

  void setActionSpace(shared_ptr<const FeatureSpace> ss){ if(!ss) return; action_space_ = ss; };
  void setStateSpace(shared_ptr<const FeatureSpace> ss){ if(!ss) return; state_space_ = ss; };
  shared_ptr<const FeatureSpace> getActionSpace() const{ return action_space_; };
  shared_ptr<const FeatureSpace> getStateSpace() const{ return state_space_; };
protected:
  std::vector<int> actions_;
  bool is_init_;
  std::string name_;

  shared_ptr<const FeatureSpace> action_space_; /**< Describes this MDP's action space. */
  shared_ptr<const FeatureSpace> state_space_; /**< Describes this MDP's state space. */
};

} /* namespace pomdp */

#endif /* POLICY_H */
