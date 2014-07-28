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
/*
 * @file
 *
 * Executable for Inverse Reinforcement Learning using a human Oracle.
 *
 * @date Jul 24, 2013
 * @author Bea Adkins
 */

#include <ros/ros.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <string>

#include "assert_type.h"
#include "command_line.h"
#include "compound_policy.h"
#include "mdp_factory.h"
#include "multinomial_policy.h"
#include "pomdp_node/oracle_irl.h"
#include "pomdp.h"
#include "pretty.h"

using namespace pomdp;

// Services
ros::ServiceServer oracle_irl_srv; // rosservice call /estimate_action shuttle 2 3

// POMDP object.
MDP* the_mdp;

// Policy object.
shared_ptr<CompoundPolicy> the_policy;

/**
 * Determines action based on belief state using named policy.
 */
bool oracleIRLCallback(pomdp_node::oracle_irl::Request &req,
                       pomdp_node::oracle_irl::Response &res)
{
  // Parse request.
  int observation = req.observation;
  int oracle_action = req.oracle_action;
  ROS_DEBUG_STREAM("\nDialogue IRL request:\no = " << the_mdp->prettyObs(observation) << ", oracle_action = " << the_mdp->prettyAction(oracle_action));

  // Step POMDPs
  int n_models = the_policy->getNumModels();
  std::vector<int> actions; // Action suggested by each policy.

  // Update belief state.
  if(!the_mdp->updateBeliefState(observation))
  {
    ROS_WARN("Couldn't update belief of '%s' using (a = %s, z = %s).",
             the_mdp->getName().c_str(), the_mdp->prettyAction(the_mdp->getLastAction()).c_str(),
             the_mdp->prettyObs(observation).c_str());
    return false;
  }
  ROS_DEBUG_STREAM("New belief state: " << the_mdp->prettyState());

  // Run policies.
  the_policy->applyPolicies(the_mdp->getState(), actions);
  for(int m = 0; m < n_models; m++)
  {
    if(actions[m] == -1)
    {
      ROS_WARN("Failed to apply policy #%i of '%s' for POMDP '%s'!",
               m, the_policy->getName().c_str(), the_mdp->getName().c_str());
      return false;
    }
  }

  // Update model likelihoods.
  std::vector<double> model_likelihoods;
  the_policy->getModelLikelihoods(model_likelihoods);
  double norm = 0; // Normalizing constant.
  for(int m = 0; m < n_models; m++)
  {
    // Apply Bayes' Rule
    // P(m|a=a_o) = P(a=a_o|m)P(m)/P(a=a_o)
    double P_a_given_m = (actions[m] == oracle_action) ? 0.95 : 0.05;
    model_likelihoods[m] = P_a_given_m * model_likelihoods[m];
    norm += model_likelihoods[m];
  }
  for(int m = 0; m < n_models; m++)
    // Normalize them.
    model_likelihoods[m] /= norm;
  if(!the_policy->setModelLikelihoods(model_likelihoods))
  {
    ROS_ERROR_STREAM("Invalid PMF: " << Pretty::vectorToString(model_likelihoods));
    return false;
  }

  // Actually use the oracle's action (not any of the policies' actions)
  the_mdp->setLastAction(oracle_action);

  // Reply
  res.model_likelihoods = model_likelihoods;
  res.model_actions = actions;
  std::vector<std::string> actions_str;
  BOOST_FOREACH(int a, actions)
  {
    actions_str.push_back(the_mdp->prettyAction(a));
  }
  ROS_DEBUG_STREAM("\nOracle IRL reply:\nactions   = " << Pretty::vectorToString(actions_str) << "\nP(models) = "
                   << Pretty::vectorToString(model_likelihoods) << std::endl);
  return true;
}

int main(int argc, char **argv)
{
  std::string node_name = "oracle_irl";
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  // Variables to help with parsing.
  std::vector<MDP*> mdps;

  //
  // Parse command line arguments
  //
  std::vector<std::string> mdp_paths;
  std::vector<std::string> policy_paths;
  if(!parseArgs(argc, argv, mdp_paths, policy_paths))
  {
    ROS_ERROR("Unable to parse command line arguments.");
    exit(EXIT_FAILURE);
  }

  //
  // Check number of files.
  //
  if(mdp_paths.size() != 1 || policy_paths.size() != 1)
  {
    ROS_ERROR_STREAM("Requires one POMDP and one CompoundPolicy. " << mdp_paths.size() << " and "
                     << policy_paths.size() << " input, respectively.");
    exit(EXIT_FAILURE);
  }

  //
  // Initialize models
  //

  // Load objects from config files.
  the_mdp = MDPFactory::loadFromFile(mdp_paths[0]);
  if(the_mdp == NULL)
  {
    ROS_ERROR("Unable to load POMDPs.");
    exit(EXIT_FAILURE);
  }

  the_policy = PolicyFactory::loadFromFile<CompoundPolicy>(policy_paths[0],
                                                           the_mdp->getStateSpace(), the_mdp->getActionSpace());
  if(!the_policy)
  {
    ROS_ERROR("Unable to load Policies.");
    exit(EXIT_FAILURE);
  }

  // Associate MDP with CompoundPolicy.
  if(!the_mdp->setMyPolicy(the_policy))
  {
    ROS_ERROR("Unable to associate policy '%s' and MDP '%s'!",
              the_policy->getName().c_str(), the_mdp->getName().c_str());
    exit(EXIT_FAILURE);
  }

  // Determine initial action (set internally within MDP).
  int a = the_mdp->applyPolicy();
  if(a == -1)
  {
    ROS_ERROR("Error applying policy '%s' to initial state of '%s'.", the_mdp->getMyPolicy()->getName().c_str(),
              the_mdp->getName().c_str());
    exit(EXIT_FAILURE);
  }
  the_mdp->setLastAction(a);
  ROS_DEBUG_STREAM("Initial action: " << the_mdp->prettyAction(a));

  // Print initial model probabilities.
  std::vector<double> model_likelihoods;
  the_policy->getModelLikelihoods(model_likelihoods);
  ROS_INFO_STREAM("Initial model likelihoods:\n" << Pretty::vectorToString(model_likelihoods));

  //
  // Advertise services.
  //
  oracle_irl_srv = n.advertiseService("oracle_irl", oracleIRLCallback);
  ROS_INFO("Listening for Oracle IRL requests...");

  ros::spin();

  return 0;
}
