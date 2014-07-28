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
l * @file
 *
 * @date Aug 16, 2013
 * @author Bea Adkins
 */

#include <boost/regex.hpp>
#include <ros/console.h>

#include "empty_mdp.h"
#include "pomdp_io.h"
#include "pretty.h"
#include "simple_feature_space.h"
#include "feature_space.h"

namespace pomdp
{

bool EmptyMDP::loadFromFile(const char* path)
{
  if(isInit())
  {
    ROS_ERROR("Unable to read %s. Requires uninitialized MDP object!", path);
    return false;
  }

  // Initialize number:name mappings or if unable, at least read number.
  shared_ptr<Pretty> pretty_action = boost::make_shared<Pretty>();
  int num_actions;
  if(pretty_action->loadNames(path, "actions"))
  {
    num_actions = pretty_action->size();
  }
  else
  {
    num_actions = getNumberFromTagColonLine(path, "actions");
  }

  shared_ptr<Pretty> pretty_state = boost::make_shared<Pretty>();
  int num_states;
  if(pretty_state->loadNames(path, "states"))
  {
    num_states = pretty_state->size();
  }
  else
  {
    num_states = getNumberFromTagColonLine(path, "states");
  }

  // @todo TODO FeatureSpace factory!
  action_space_.reset(new SimpleFeatureSpace(num_actions, pretty_action));
  ROS_DEBUG_STREAM("Added action space: " << *action_space_); // Dereferencable because just assigned!

  state_space_.reset(new SimpleFeatureSpace(num_states, pretty_state));
  ROS_DEBUG_STREAM("Added state space: " << *state_space_); // Dereferencable because just assigned!

  // Find starting state
  std::string state_start_str = getStringFromTagColonLine(path, "start");
  int state_start = Pretty::readNameOrNum(state_start_str, pretty_state);
  if(state_start == -1)
  {
    ROS_ERROR("No starting state.");
    return false;
  }
  state_cur_ = boost::make_shared<SimpleFeatureValue>(state_start);

  // Check validity of action/state counts.
  if(action_space_->getNumValues() == 0)
  {
    ROS_ERROR("Invalid number of action %i.", action_space_->getNumValues());
    return false;
  }
  if(state_space_->getNumValues() == 0)
  {
    ROS_ERROR("Empty StateSpace.");
    return false;
  }

  // Success
  this-> is_init_ = true;
  return true;
}

} /* namespace pomdp */
