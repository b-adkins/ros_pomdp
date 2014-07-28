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

#include <algorithm>
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <sstream>

#include "alpha_policy.h"
#include "assert_rng.h"
#include "assert_type.h"
#include "pomdp.h"
#include "pomdp_io.h"
#include "pretty.h"
#include "feature_space.h"
#include "belief_feature_value.h"

namespace pomdp
{

AlphaPolicy::AlphaPolicy()
{
}

AlphaPolicy::AlphaPolicy(const char* path)
{
  AlphaPolicy();
  loadFromFile(path);
}

AlphaPolicy::~AlphaPolicy()
{
}

/**
 * Loads a policy file.
 *
 * @return True on success.
 */
bool AlphaPolicy::loadFromFile(const char* path)
{
  int action;
  double alpha;
  std::vector<double> alphas;
  std::string line;
  bool alpha_line = false; // True for alpha line, false for action line. (Simple state machine.)

  // Open file.
  std::ifstream policy_file;
  policy_file.open(path);
  if(!policy_file.good())
  {
    ROS_ERROR_STREAM("Unable to open file " << path);
    return false;
  }

  // Check Policy has what it needs to read the file.
  ROS_ASSERT_MSG(state_space_, "Requires StateSpace to be initialized!");

  // Read file.
  while(!policy_file.eof())
  {
    std::getline(policy_file, line);
    if(policy_file.fail())
      continue;

    line = stripComment(line);
    if(isBlank(line))
      continue; // Skip line

    std::stringstream line_ss(line);

    // Action line
    if(!alpha_line)
    {
      // Read element from line.
      std::string action_str;
      line_ss >> action_str;
      if(line_ss.fail())
      {
        ROS_ERROR_STREAM("Invalid action line: '" << line << "'");
        return false;
      }

      // Convert to action.
      // Kludgy. @todo Change actions from ints to FeatureValues.
      shared_ptr<FeatureValue> action_obj = action_space_->readFeatureValue(action_str);
      action = boost::dynamic_pointer_cast<SimpleFeatureValue>(action_obj)->asInt();
      if(!action_space_->isValid(SimpleFeatureValue(action)))
      {
        ROS_ERROR_STREAM("Invalid action: '" << action_str << "'");
        return false;
      }

      actions_.push_back(action);
      alpha_line = true;
    }
    // Alpha line
    else if(alpha_line)
    {
      alphas.clear();
      while(line_ss >> alpha)
      {
        if(line_ss.fail())
        {
          ROS_ERROR_STREAM("Invalid alpha vector, needs decimal numbers in line: '" << line << "'");
          return false;
        }
        alphas.push_back(alpha);
      }
      alphas_.push_back(alphas);
      alpha_line = false;
    }
  }

  // Fail on empty policy.
  if(alphas_[0].size() == 0)
  {
    ROS_ERROR("Empty alpha line(s) in policy file.");
    return false;
  }
  if(actions_.size() == 0 || alphas_.size() == 0)
  {
    ROS_ERROR("Empty policy file.");
    return false;
  }

  // Fail if actions-alphas aren't one-to-one.
  if(actions_.size() != alphas_.size())
  {
    ROS_ERROR_STREAM("Mismatch between number of actions (" << actions_.size()
                     << ") and number of alpha vectors (" << alphas_.size() << ").");
    return false;
  }

  // Fail if alphas inconsistent in size.
  for(int a = 0; a < alphas_.size() - 1; a++)
  {
    if(alphas_[a].size() != alphas_[a + 1].size())
    {
      ROS_ERROR("Inconsistently sized alpha vectors:");
      ROS_ERROR_STREAM(Pretty::vectorToString(alphas_[a]));
      ROS_ERROR_STREAM(Pretty::vectorToString(alphas_[a + 1]));
      return false;
    }
  }

  // Fail on mismatched number of states.
  unsigned int num_states = alphas_[0].size();
  if(num_states != state_space_->getNumValues())
  {
    ROS_ERROR_STREAM("Number of states read (" << num_states << ") does not match number in");
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
 * Determines action from belief state.
 *
 * @param belief_state Vector of probabilities, one for each state.
 * @return Action or -1 for failure.
 */
int AlphaPolicy::applyPolicy(const FeatureValue& belief) const
{
  if(!isInit())
    return -1;
  ROS_ASSERT_CMD(actions_.size() > 0, return -1);
  ROS_ASSERT_TYPE(belief, BeliefFeatureValue);
//  ROS_ASSERT_TYPE_CMD(belief, BeliefState, return -1);
  ROS_ASSERT((state_space_->isValid(belief)));
//  ROS_ASSERT_CMD((state_space_->isValid(belief)), return -1);

  const BeliefFeatureValue& b = dynamic_cast<const BeliefFeatureValue&>(belief);

  // Multiply belief state by every alpha vector
  std::vector<double> values(actions_.size(), 0.0);
  for(int n = 0; n < actions_.size(); n++)
  {
    for(int s = 0; s < getNumStates(); s++)
    {
      values[n] += b[s] * alphas_[n][s];
    }
  }

  // Debug printing
  ROS_DEBUG_STREAM("Policy '" << getName() << "' values: " << Pretty::vectorToString(values));

  // Find highest value and return corresponding action
  std::vector<double>::iterator value_max;
  value_max = std::max_element(values.begin(), values.end());
  int i_a = std::distance(values.begin(), value_max);
  return actions_[i_a];
}

} /* namespace pomdp */
