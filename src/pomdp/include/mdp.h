/**
 * @class pomdp::MDP
 *
 * Markov Decision Process class.
 *
 * Multiple MDPs can be active at once. Use child class for POMDPs.
 *
 * In addition to the constructor, MDP::MDP(), MDP::loadFromFile() must be called to initialize the MDP from a config
 * file.
 *
 * @date Jul 18, 2013
 * @author Bea Adkins
 */

#ifndef MDP_HPP
#define MDP_HPP

#include <boost/shared_ptr.hpp>
#include <ros/assert.h>
#include <ros/console.h>
extern "C"
{
#include <mdp/mdp.h>
#include <mdp/mdp-common.h>
}
#include <exception>
#include <string>
#include <vector>

extern "C"
{
#include "mdp_util.h"
}

#include "assert_rng.h"
#include "mdp_policy.h"
#include "simple_feature_value.h"

using boost::shared_ptr; // Because it's long and ugly enough v.s. a '*'.

namespace pomdp
{
// No need to tangle dependencies.
class Policy;
class Pretty;
class FeatureSpace;

class MDP
{
public:
  MDP();
  MDP(const char* path);
  MDP(std::string path);
  virtual ~MDP();
  virtual bool loadFromFile(const char* path);
  bool loadFromFile(const std::string& path);

  static MDP* findMDP(const std::vector<MDP*>& pomdps, std::string name);

  Matrix getTransMatrix(int action);
  double lookupTransProb(const SimpleFeatureValue& state_old, const SimpleFeatureValue& state_new, int action);
  bool estimateAction(int state_old, int state_new, std::vector<int> &actions, std::vector<double> &trans_probs);

  virtual bool updateBeliefState(int observation);

  /** Getter for associated policy. @return Associated policy. */
  virtual shared_ptr<const Policy> getMyPolicy() const { return my_policy_; };
  virtual bool setMyPolicy(shared_ptr<const Policy> p);
  virtual int applyPolicy();

  bool isInit() const{ return this->is_init_; }
  int getNumActions() const;
  int getLastAction() const { return last_action_; }
  void setLastAction(int a){ last_action_ = a; }; //@todo remove! Replace with polymorphic accessess via Policies
  std::string getName() const { return name_; }
  void setName(std::string name){ this->name_ = name; }
  FeatureValue const& getState() const { return *state_cur_; } /**< @return Constant reference to belief state. */
  FeatureValue& getState(){ return *state_cur_; } /**< @return Copy of belief state. */

  bool setActionSpace(shared_ptr<const FeatureSpace> ss);
  shared_ptr<const FeatureSpace> getActionSpace(){ return action_space_; };
  std::string prettyLastAction() const;
  std::string prettyAction(int action) const;
  bool setStateSpace(shared_ptr<const FeatureSpace> ss);
  shared_ptr<const FeatureSpace> getStateSpace(){ return state_space_; };
  std::string prettyState() const;
  std::string prettyState(const FeatureValue& state) const;
  virtual std::string prettyObs(int observation) const;
protected:
  // Analogs to <mdp.h> global variables.
  Matrix *P_;  /**< Transition Probabilities */

  // MDP Variables
  int last_action_; /**< Last action performed. */
  shared_ptr<FeatureValue> state_cur_; /**< Current state. */

  // Miscellaneous class variables.
  bool  is_init_;
  std::string name_;
  shared_ptr<const FeatureSpace> action_space_; /**< Describes this MDP's action space. */
  shared_ptr<const FeatureSpace> state_space_; /**< Describes this MDP's state space. */
private:
  // Associated policy with POMDP.
  shared_ptr<const MDPPolicy> my_policy_;
};

} /* namespace pomdp */

#endif /* MDP_HPP */
