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
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include "assert_type.h"
#include "map_policy.h"
#include "pomdp_io.h"
#include "pretty.h"
#include "simple_feature_value.h"
#include "feature_space.h"

namespace pomdp
{

MapPolicy::MapPolicy()
{
}

MapPolicy::MapPolicy(const char* path)
{
  loadFromFile(path);
}

MapPolicy::~MapPolicy()
{
}

/**
 * Loads a policy file.
 *
 * @return True on success.
 */
bool MapPolicy::loadFromFile(const char* path)
{
  if(!action_space_)
    ROS_WARN("No action space set! Policy values will not be validated.");

  int action;
  actions_.clear();

  // Open file.
  std::ifstream policy_file;
  policy_file.open(path);
  if(!policy_file.good())
  {
    ROS_ERROR("Unable to open policy file '%s'.", path);
    return false;
  }

  // Check Policy has what it needs to read the file.
  ROS_ASSERT_MSG(state_space_, "Requires StateSpace to be initialized!");

  // Read file.
  std::string line;
  while(!policy_file.eof())
  {
    std::getline(policy_file, line);
    if(policy_file.fail())
      continue;

    line = stripComment(line);
    if(isBlank(line))
      continue;

    // Read actions.
    std::stringstream line_ss(line);
    std::string action_str;
    line_ss >> action_str;
    if(!line_ss.fail())
    {
      if(!action_space_)
      {
        std::stringstream(action_str) >> action;
        actions_.push_back(action);
      }
      else
      {
        shared_ptr<FeatureValue> action_obj = action_space_->readFeatureValue(action_str);
        action = boost::dynamic_pointer_cast<SimpleFeatureValue>(action_obj)->asInt();
        if(action_space_->isValid(SimpleFeatureValue(action)))
          actions_.push_back(action);
      }
    }
  }

  int num_states = actions_.size();

  // Fail on empty policy.
  if(num_states == 0)
  {
    ROS_ERROR("Empty policy file '%s'.", path);
    return false;
  }

  // Fail on mismatched number of states. No need to check for NULL state_space_, already checked above.
  if(num_states != state_space_->getNumValues())
  {
    ROS_ERROR_STREAM("Number of states in policy (" << num_states << ") does not match number in ");
    ROS_ERROR_STREAM(*state_space_);
    return false;
  }

  // Use filename for name
  boost::filesystem::path bpath(path);
  setName(bpath.stem().string());

  is_init_ = true;
  return true;
}

/**
 * Determines action from current state.
 *
 * @param index Current state, within [0, num_states).
 * @return Action or -1 for failure.
 */
int MapPolicy::applyPolicy(const FeatureValue& index) const
{
  if(!isInit())
    return -1;
  ROS_ASSERT_CMD(state_space_->isValid(index), return -1);

//  try
//  {
//    const FeatureVectorValue& v = dynamic_cast<const FeatureVectorValue&>(index);
//    state = boost::make_shared<State>(&state_space_->toNumericState(v));
//  }
//  catch(std::bad_cast& e){}

  ROS_ASSERT_TYPE(index, SimpleFeatureValue);
  const SimpleFeatureValue& state = dynamic_cast<const SimpleFeatureValue&>(index);
  return actions_[state.asInt()];
}

} /* namespace pomdp */
