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
 * Focused node for running a POMDP.
 *
 * @date Jul 20, 2013
 * @author B. Adkins
 */

#include <boost/algorithm/string/replace.hpp>
#include <boost/units/detail/utility.hpp>
#include <ros/ros.h>
#include <vector>

#include "alpha_policy.h"
#include "command_line.h"
#include "feature_space_factory.h"
#include "mdp_factory.h"
#include "policy.h"
#include "policy_factory.h"
#include "pomdp.h"
#include "pomdp_node/action.h"
#include "pomdp_node/observation.h"
#include "pretty.h"
#include "simple_feature_space.h"
#include "vector_feature_space.h"

using namespace pomdp;

// Publish-subscribe
std::string action_pub_name;
ros::Publisher action_pub;
std::string obs_sub_name;
ros::Subscriber obs_sub;

// (PO)MDP object.
MDP* the_mdp;

/**
 * Current failure behavior: doesn't publish anything to action topic.
 */
void observationCallback(const pomdp_node::observation& req)
{
  // Parse state.
  SimpleFeatureValue observation(req.observation);
  ROS_DEBUG_STREAM("Received observation " << the_mdp->prettyObs(observation.asInt()));

  // Update belief state
  if(!the_mdp->updateBeliefState(observation.asInt()))
  {
    ROS_WARN_STREAM("Error updating state, keeping old state: " << the_mdp->prettyState());
    return;
  }
  ROS_DEBUG_STREAM("Belief state: " << the_mdp->prettyState());

  // Apply the policy
  int action_new = the_mdp->applyPolicy();
  if(action_new == -1)
  {
    ROS_ERROR("Error applying policy for o = %s, a = %s, b = %s",
              the_mdp->prettyObs(observation.asInt()).c_str(), the_mdp->prettyAction(the_mdp->getLastAction()).c_str(),
              the_mdp->prettyState().c_str());
    return;
  }

  // Send reply
  ROS_DEBUG_STREAM("Published action " << the_mdp->prettyAction(action_new) << std::endl);
  pomdp_node::action rsp;
  rsp.action = action_new;
  action_pub.publish(rsp);
}

int main(int argc, char **argv)
{
  std::string name = "pomdp_run";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;

  //
  // Parse command line arguments
  //
  std::vector<std::string> mdp_paths;
  std::vector<std::string> policy_paths;
  shared_ptr<Policy> the_policy;
  if(!parseArgs(argc, argv, mdp_paths, policy_paths))
  {
    ROS_ERROR("Unable to parse command line arguments.");
    exit(EXIT_FAILURE);
  }

  // Check for single POMDP and Policy.
  if(mdp_paths.size() < 1 || policy_paths.size() < 1)
  {
    ROS_ERROR("This requires a single (PO)MDP and a single Policy.");
    exit(EXIT_FAILURE);
  }
  else if(mdp_paths.size() > 1 || policy_paths.size() > 1)
  {
    ROS_ERROR("This node only supports a single (PO)MDP and Policy.");
    exit(EXIT_FAILURE);
  }

  //
  // Load complex StateSpace from param server, if path is available.
  //
  // Will use a SimpleStateSpace extracted from MDP file if none is found.
  //
  FeatureSpaceFactory fs_fact;
  shared_ptr<FeatureSpace> state_space;
  std::string state_space_path;
  bool hasAlternateStateSpace = false;
  if(nh.getParam("state_space_path", state_space_path))
  // Only does anything on success.
  {
    hasAlternateStateSpace = true;

    // Construct state space from given file.
    state_space = fs_fact.loadFromFile(state_space_path, "state_space");
    ROS_INFO_STREAM(*state_space);
  }

  //
  // Read POMDP files.
  //
  the_mdp = MDPFactory::loadFromFile(mdp_paths[0]);
  if(the_mdp == NULL)
  {
    ROS_ERROR("Unable to load (PO)MDP '%s'.", mdp_paths[0].c_str());
    exit(EXIT_FAILURE);
  }

  //
  // Replace (PO)MDP's StateSpace if a more complex one was loaded.
  //
  if(state_space && state_space->isInit())
  {
    if(the_mdp->setStateSpace(state_space))
    {
      ROS_INFO_STREAM("Replaced (PO)MDP \"" << the_mdp->getName() << "\"'s StateSpace with:");
      ROS_INFO_STREAM(*state_space); // Null check above
    }
    else
    {
      ROS_ERROR_STREAM("Unable to use " << *state_space);
      ROS_ERROR_STREAM("with MDP \"" << the_mdp->getName() << "\"");
      exit(EXIT_FAILURE);
    }
  }
  // Fail if there was a state space but it couldn't be loaded.
  else if(hasAlternateStateSpace && !state_space->isInit())
  {
    ROS_ERROR_STREAM("Unable to load StateSpace from state_space_path \"" << state_space_path << "\"");
    exit(EXIT_FAILURE);
  }
  // Push a simple state space to the param server.
  else
  {
    shared_ptr<const SimpleFeatureSpace> simple_state_space
      = boost::dynamic_pointer_cast<const SimpleFeatureSpace>(the_mdp->getStateSpace());

    // Exists and is a SimpleFeatureSpace
    if(simple_state_space)
    {
      XmlRpc::XmlRpcValue state_space_xml;
      state_space_xml << *simple_state_space;
      nh.setParam("state_space", state_space_xml);
    }
  }

  // Push a simple action space to the param server.
  {
    shared_ptr<const SimpleFeatureSpace> simple_action_space
    = boost::dynamic_pointer_cast<const SimpleFeatureSpace>(the_mdp->getActionSpace());

    // Exists and is a SimpleFeatureSpace
    if(simple_action_space)
    {
      XmlRpc::XmlRpcValue action_space_xml;
      action_space_xml << *simple_action_space;
      nh.setParam("action_space", action_space_xml);
    }
  }

  // Debug print.
  ROS_DEBUG_STREAM("Initial state: " << the_mdp->prettyState());

  //
  // Read policy file.
  //
  the_policy = PolicyFactory::loadFromFile<Policy>(policy_paths[0],
                                                   the_mdp->getStateSpace(), the_mdp->getActionSpace());
  if(!the_policy)
  {
    ROS_ERROR("Unable to load Policy '%s'.", policy_paths[0].c_str());
    exit(EXIT_FAILURE);
  }
  the_policy->setName(name);

  //
  // Associate POMDP with Policy.
  //
  if(!the_mdp->setMyPolicy(the_policy))
  {
    ROS_ERROR("Unable to associate policy '%s' and POMDP '%s'!",
              the_policy->getName().c_str(), the_mdp->getName().c_str());
    exit(EXIT_FAILURE);
  }

  //
  // Determine initial action (set internally within POMDP).
  //
  int a = the_mdp->applyPolicy();
  if(a == -1)
  {
    ROS_ERROR("Error applying policy '%s' to initial state of '%s'.", the_policy->getName().c_str(), the_mdp->getName().c_str());
    exit(EXIT_FAILURE);
  }
  ROS_DEBUG_STREAM("Initial action: " << the_mdp->prettyAction(a));

  //
  // Advertise topics.
  //

  // Create names (default).
//  action_pub_name = ros::this_node::getName() + "/" + DEFAULT_ACTION_TOPIC;
//  obs_sub_name = ros::this_node::getName() +  "/" + DEFAULT_OBSERVATION_TOPIC;
    action_pub_name = DEFAULT_ACTION_TOPIC;
    obs_sub_name = DEFAULT_OBSERVATION_TOPIC;

  // Advertise.
  action_pub = nh.advertise<pomdp_node::action>(action_pub_name, DEFAULT_ACTION_QUEUE);
  if(!action_pub)
    ROS_ERROR_STREAM("Unable to initialize topic " << action_pub);
  else
    ROS_INFO_STREAM("Ready to publish actions on " << action_pub.getTopic());


  obs_sub = nh.subscribe(obs_sub_name, DEFAULT_OBSERVATION_QUEUE, observationCallback);
  if(!obs_sub)
    ROS_ERROR_STREAM("Unable to subscribe to " << obs_sub_name);
  else
    ROS_INFO_STREAM("Listening for observations on " << obs_sub.getTopic());

  ros::spin();

  return 0;
}

