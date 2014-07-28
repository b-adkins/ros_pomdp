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
 * Executable for Inverse Reinforcement Learning using a dialogue system.
 *
 * Requires the following values in the param server:
 * - cfg_root - Root of config file paths. (All other paths relative to cfg_root.)
 * - pomdp_paths - A single (PO)MDP config file.
 * - policy_paths - A single CompoundPolicy config file.
 * - bag_paths - Bags of words, one per feature value.
 *
 * Optional params:
 * - learning_rate
 *
 * @date Jul 25, 2013
 * @author Bea Adkins
 */

#include <boost/lexical_cast.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <sstream>

#include "roscpp_param_patch.h"

#include "alpha_policy.h"
#include "assert_type.h"
#include "bag_of_words_observation_model.h"
#include "command_line.h"
#include "compound_policy.h"
#include "feature_space_factory.h"
#include "mdp.h"
#include "mdp_factory.h"
#include "mdp_policy.h"
#include "policy_factory.h"
#include "pomdp_node/action.h"
#include "pomdp_node/dialogue_irl.h"
#include "pomdp.h"
#include "pretty.h"
#include "simple_feature_space.h"
#include "vector_feature_space.h"

using namespace pomdp;

// Services
ros::ServiceServer dialogue_irl_srv;

// Topics
ros::Publisher model_states_pub;
ros::Publisher oracle_action_pub;

// POMDP objects.
MDP* the_mdp;
shared_ptr<CompoundPolicy> the_policy;

// Bag of words, mapping phrase to action
BagOfWordsObservationModel bag_of_words;

// @todo Refactor these constants to an InverseReinforcementLearner class

// @todo Come up with a better name for this variable
// How fast model probabilies change when their actions match/don't match oracle's action.
double learning_rate;

/**
 * Determines action based on belief state using named policy.
 */
bool dialogueIRLCallback(pomdp_node::dialogue_irl::Request &req,
                         pomdp_node::dialogue_irl::Response &res)
{
  // Parse request.
  int observation = req.observation;
  std::vector<std::string> phrase = req.oracle_phrase;
  ROS_DEBUG_STREAM("\nDialogue IRL request:\no = " << the_mdp->prettyObs(observation) << ", phrase = " << Pretty::vectorToString(phrase));

  // Oracle must have said something to be used
  bool oracle_spoke = false;
  if(!phrase.empty())
    oracle_spoke = true;

  // Step POMDPs
  int n_models = the_policy->getNumModels();
  std::vector<int> actions(n_models); // Action suggested by each policy.

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

  int oracle_action = -1; // Default: oracle said nothing
  std::vector<double> model_likelihoods;
  the_policy->getModelLikelihoods(model_likelihoods);

  // Oracle must have said something to be used
  if(oracle_spoke)
  {
    // Get likelihood P(a|ph) for all actions.
    std::vector<double> P_as_given_phrase;
    if(!bag_of_words.getProbActionForPhrase(phrase, P_as_given_phrase))
    {
      ROS_ERROR("Error getting P(a|ph).");
      return false;
    }
    ROS_DEBUG_STREAM("Oracle said: " << Pretty::vectorToString(phrase));

    // Update model likelihoods.
    double norm = 0; // Normalizing constant.
    for(int m = 0; m < n_models; m++)
    {
      // Apply Bayes' Rule
      // P(m|phrase) = SUM{ P(m|a) P(a|phrase) } P(m)/P(phrase)
      double sum = 0;
      for(int a = 0; a < the_mdp->getNumActions(); a++)
      {
        double P_a_given_phrase = P_as_given_phrase[a];
        double P_m_given_a = (actions[m] == a) ? learning_rate : 1 - learning_rate;
        sum += P_m_given_a * P_a_given_phrase;
      }
      model_likelihoods[m] = sum * model_likelihoods[m];
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

    // Use the oracle's action (most likely one).
    // @todo Find a real argmax function or refactor this
    int likeliest_action = 0;
    for(int a = 1; a < P_as_given_phrase.size(); a++)
      if(P_as_given_phrase[a] > P_as_given_phrase[likeliest_action])
        likeliest_action = a;

    oracle_action = likeliest_action; // Converts iterator to index number
    ROS_DEBUG_STREAM("Oracle action: " << oracle_action);
    
    // @todo THIS DOES NOT REFLECT REALITY. NEEDS TO KNOW WHICH ACTION WAS ACTUALLY IMPLEMENTED TO WORK WITH A NON-EMPTY (PO)MDP
    the_mdp->setLastAction(oracle_action);
  }

  // Reply
  std::vector<std::string> model_names;
  the_policy->getModelNames(model_names);
  pomdp_node::model_outputs model_states;
  for(int m = 0; m < n_models; m++)
  {
    pomdp_node::model_output model_state;
    model_state.name = model_names[m];
    model_state.index = m;
    model_state.action = actions[m];
    model_state.likelihood = model_likelihoods[m];
    model_states.data.push_back(model_state);
  }
  res.model_states = model_states;
  res.oracle_action = oracle_action;

  // Publish to logging topics
  model_states_pub.publish(model_states);
  if(oracle_spoke)
  {
    pomdp_node::action oracle_action_msg;
    oracle_action_msg.action = oracle_action;
    oracle_action_pub.publish(oracle_action_msg);
  }

  // Pretty print actions
  std::vector<std::string> actions_str;
  BOOST_FOREACH(int a, actions)
  {
    actions_str.push_back(the_mdp->prettyAction(a));
  }
  ROS_DEBUG_STREAM("\nDialogue IRL reply:\noracle_action = " << oracle_action <<
                   "\nactions   = " << Pretty::vectorToString(actions_str) << "\nP(models) = "
                   << Pretty::vectorToString(model_likelihoods) << std::endl);
  return true;
}

#define NO_PARAM_ABORT(param) \
  {                           \
  ROS_ERROR("'%s' not found in param server!", nh.resolveName(param).c_str());\
  return false; }

/**
 * Obtains dialogue_irl parameters from the ROS parameter server.
 *
 * Uses names relative to the node. Does not open files! Only checks for
 * parameter number and existence!
 *
 * @param[in] Node handle for current node.
 * @param[out] cfg_root Root path to which all other paths will be concatenated.
 * @param[out] mdp_paths Paths to POMDP files.
 * @param[out] policy_paths Paths to Policy files.
 * @param[out] bags_of_words Paths to bags of words for each action.
 * @param[out] learning_rate How fast model probabilies change when their actions match/don't match oracle's action.
 * @return True on success.
 */
bool getDialogueIRLParameters(ros::NodeHandle& nh, std::string& cfg_root,
                              std::vector<std::string>& mdp_paths, std::vector<std::string>& policy_paths,
                              std::vector<std::string>& bags_of_words, double& learning_rate)
{
//  using namespace ros; // If patch is officially added.
  using namespace rospatch; // Until then

  // Official ros function
  if(!nh.getParam("cfg_root", cfg_root))
    NO_PARAM_ABORT("cfg_root");

  // Functions from potential patch to ros.
  if(!nh.getParam("pomdp_paths", mdp_paths))
    NO_PARAM_ABORT("pomdp_paths");
  if(!nh.getParam("policy_paths", policy_paths))
    NO_PARAM_ABORT("policy_paths");
  if(!nh.getParam("bag_paths", bags_of_words))
    NO_PARAM_ABORT("bag_paths");

  // Default value
  learning_rate = 0.90;
  double learning_rate_tmp;
  if(nh.getParam("learning_rate", learning_rate_tmp))
  {
    // @todo Refactor this check to IRL class.
    if(learning_rate_tmp <= 0 || learning_rate_tmp >= 1)
    {
      ROS_WARN_STREAM("learning_rate must be a probability between 0 and 1! Using default value of " << learning_rate);
    }
    else
    {
      learning_rate = learning_rate_tmp;
      ROS_DEBUG_STREAM("Using learning rate of " << learning_rate);
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_irl");
  ros::NodeHandle nh;

  // Variables to help with parsing.
  std::string cfg_root;
  std::vector<std::string> mdp_paths;
  std::vector<std::string> policy_paths;
  std::vector<std::string> bag_paths; // Bags of words

  std::vector<MDP*> mdps;
  std::vector<shared_ptr<POMDPPolicy> > policies;

  //
  // Get paths from param server.
  //
  if(!getDialogueIRLParameters(nh, cfg_root, mdp_paths, policy_paths, bag_paths, learning_rate))
  {
    ROS_ERROR("Unable to read config from parameter server.");
    exit(EXIT_FAILURE);
  }

  // Convert cfg_root and relative paths to absolute paths.
  namespace bfs = boost::filesystem;
  for(int m = 0; m < mdp_paths.size(); m++)
  {
    mdp_paths[m] = (bfs::path(cfg_root) / bfs::path(mdp_paths[m])).string();
    policy_paths[m] = (bfs::path(cfg_root) / bfs::path(policy_paths[m])).string();
  }
  for(std::vector<std::string>::iterator itr = bag_paths.begin();
      itr != bag_paths.end();
      itr++)
  {
    *itr = (bfs::path(cfg_root) / bfs::path(*itr)).string();
  }

  // Check number of models.
  if(mdp_paths.size() != 1 || policy_paths.size() != 1)
  {
    ROS_ERROR_STREAM("Requires one POMDP and one CompoundPolicy. " << mdp_paths.size() << " and "
                     << policy_paths.size() << " input, respectively.");
    exit(EXIT_FAILURE);
  }

  //
  // Initialize models
  //

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

  // Load policy from file
  the_policy = PolicyFactory::loadFromFile<CompoundPolicy>(policy_paths[0],
                                                           the_mdp->getStateSpace(), the_mdp->getActionSpace());
  if(!the_policy)
  {
    ROS_ERROR("Unable to load Policy '%s'.", policy_paths[0].c_str());
    exit(EXIT_FAILURE);
  }

  // Check object types.
  for(int p = 0; p < mdps.size(); p++)
  {
    MDP& current_mdp = *(mdps[p]); // Rename to make the assert message cleaner.
    ROS_ASSERT_TYPE(current_mdp, POMDP);
  }

  // Associate MDP with CompoundPolicy.
  if(!the_mdp->setMyPolicy(the_policy))
  {
    ROS_ERROR("Unable to associate policy '%s' and MDP '%s'!",
              the_policy->getName().c_str(), the_mdp->getName().c_str());
    exit(EXIT_FAILURE);
  }

  //
  // Determine initial action (set internally within POMDP).
  //
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
  // Load bags of words.
  //
  if(!bag_of_words.loadModel(bag_paths))
  {
    std::stringstream bag_paths_ss;
    for(int i = 0; i < bag_paths.size(); i++)
      bag_paths_ss << "'" << bag_paths[i] << "'\n";

    ROS_ERROR_STREAM("Unable to load bags of words:\n" << bag_paths_ss.str());
    exit(EXIT_FAILURE);
  }

  //
  // Advertise services.
  //
  dialogue_irl_srv = nh.advertiseService("dialogue_irl", dialogueIRLCallback);
  ROS_INFO("Listening for Dialogue IRL requests...");

  //
  // Advertise topics.
  //

  // Don't exit on failure because these are for logging.
  model_states_pub = nh.advertise<pomdp_node::model_outputs>("model_states", 1);
  if(!model_states_pub)
    ROS_ERROR_STREAM("Unable to initialize topic " << model_states_pub);
  else
    ROS_INFO_STREAM("Ready to publish actions on " << model_states_pub.getTopic());

  oracle_action_pub = nh.advertise<pomdp_node::action>("oracle_action", 1);
  if(!oracle_action_pub)
    ROS_ERROR_STREAM("Unable to initialize topic " << oracle_action_pub);
  else
    ROS_INFO_STREAM("Ready to publish actions on " << oracle_action_pub.getTopic());


  ros::spin();

  return 0;
}
