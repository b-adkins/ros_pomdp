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
 * @class pomdp::PolicyFactory
 *
 * Class for creating Policies. The only source file a developer has to modify to add a new policy type to an existing
 * node!
 *
 * @date Aug 9, 2013
 * @author Bea Adkins
 */

#ifndef POLICYFACTORY_H
#define POLICYFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/units/detail/utility.hpp>
#include <map>
#include <ros/console.h>
#include <string>
#include <typeinfo>

#include "pretty.h"

using boost::shared_ptr; // Because it's long and ugly enough v.s. a '*'.

namespace pomdp
{
// No need to tangle dependencies.
class Policy;
class FeatureSpace;

class PolicyFactory
{
public:
  /**
   * Polymorphically initializes a Base_Policy from a config file.
   *
   * @param tparam Base_Policy Loaded Policy is required to be this type or a child.
   * @param path Path to config file. Determines policy type.
   * @param state_space Smart pointer to StateSpace for this Policy.
   * @param action_map Optional name:number mapping for actions.
   * @return Smart pointer to new Base_Policy object. Empty (boolean value of false) on failure.
   */
  template<class Base_Policy>
  static shared_ptr<Base_Policy> loadFromFile(const char* path, shared_ptr<const FeatureSpace> state_space,
                                              shared_ptr<const FeatureSpace> action_space)
  {
    std::string path_cpp(path);
    return loadFromFile<Base_Policy>(path_cpp, state_space, action_space);
  }
  template<class Base_Policy>
  static shared_ptr<Base_Policy> loadFromFile(const std::string& path, shared_ptr<const FeatureSpace> state_space,
                                              shared_ptr<const FeatureSpace> action_space);
  template<class Base_Policy>
  static bool loadFromFiles(const std::vector<std::string>& paths, std::vector<shared_ptr<Base_Policy> >& policies,
                            shared_ptr<const FeatureSpace> state_space,
                            shared_ptr<const FeatureSpace> action_space);

private:
  static shared_ptr<Policy> loadFromFileImpl(const std::string& path,
                                             shared_ptr<const FeatureSpace> state_space,
                                             shared_ptr<const FeatureSpace> action_space_);
};

/**
 * Polymorphically initializes a Base_Policy from a config file.
 *
 * @param tparam Base_Policy Loaded Policy is required to be this type or a child.
 * @param path Path to config file. Determines policy type.
 * @param state_space Smart pointer to StateSpace for this Policy.
 * @param action_map Optional name:number mapping for actions.
 * @return Smart pointer to new Base_Policy object. Empty (boolean value of false) on failure.
 */
template<class Base_Policy>
shared_ptr<Base_Policy> PolicyFactory::loadFromFile(const std::string& path, shared_ptr<const FeatureSpace> state_space,
                                                    shared_ptr<const FeatureSpace> action_space)
{
  // Type check. (Returns empty on failure.)
  return boost::dynamic_pointer_cast<Base_Policy>(loadFromFileImpl(path, state_space, action_space));
}

/**
 * Polymorphically initializes one or more Base_Policy objects from a list of config files.
 *
 * @tparam Base_Policy Loaded Policy is required to be this type or a child.
 * @param[in] paths Vector of config file paths.
 * @param[in/out] policies Vector of smart pointers to initialized Base_Policy objects. New Base_Policy objects appended
 *                           to existing contents on success.
 * @param[in] state_space Smart pointer to StateSpace for these Policies.
 * @param[in] action_map Optional name:number mapping for actions.
 * @return True on success.
 *
 * @todo Any reason to keep this around? Why not just use a for loop?
 */
template<class Base_Policy>
bool PolicyFactory::loadFromFiles(const std::vector<std::string>& paths, std::vector<shared_ptr<Base_Policy> >& policies,
                                  shared_ptr<const FeatureSpace> state_space,
                                  shared_ptr<const FeatureSpace> action_space)
{
  // Stored initialized policies so nothing is added if call fails.
  std::vector<shared_ptr<Base_Policy> > tmp_policies;

  // State Space is mandatory! (name:number mapping inside is optional)
  if(!state_space)
  {
    ROS_ERROR("StateSpace required to initialize Policies!");
    return false;
  }

  namespace bud = boost::units::detail;
  for(int i = 0; i < paths.size(); i++)
  {
    const std::string& pol_path = paths[i];
    shared_ptr<Base_Policy> policy_new = PolicyFactory::loadFromFile<Base_Policy>(pol_path, state_space, action_space);
    if(!policy_new)
    {
      // Temporary policies deallocated when shared_ptr s go out of scope.
      return false;
    }

    tmp_policies.push_back(policy_new);
  }

  // Append new policies to list.
  policies.insert(policies.end(), tmp_policies.begin(), tmp_policies.end());
  return true;
}

} /* namespace pomdp */
#endif /* POLICYFACTORY_H */
