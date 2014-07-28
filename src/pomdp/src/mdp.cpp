/**
 * @file
 *
 *
 *
 * @date Jul 18, 2013
 * @author Bea Adkins
 */

#include "mdp.h"

#include "assert_type.h"
#include "policy.h"
#include "pomdp_io.h"
#include "pretty.h"
#include "simple_feature_space.h"
#include "feature_space.h"
#include "simple_feature_value.h"

namespace pomdp
{

/**
 * Does not allocate any of the POMDP variables!
 */
MDP::MDP() : is_init_(false), P_(NULL), last_action_(0), my_policy_(), state_cur_(), action_space_(), state_space_()
{
}

/**
 * Reads a .POMDP file into an uninitialized MDP object.
 *
 * POMDP will be initialized on success.
 *
 * Uses A. Cassandra's .POMDP format.
 *
 * @return True on success.
 */
bool MDP::loadFromFile(const std::string& path)
{
  return loadFromFile(path.c_str());
}

/**
 * Reads a .POMDP file into an uninitialized MDP object.
 *
 * POMDP will be initialized on success.
 *
 * Uses A. Cassandra's .POMDP format.
 *
 * @return True on success.
 */
bool MDP::loadFromFile(const char* path)
{
  if(isInit())
  {
    ROS_ERROR("Unable to read %s. Requires uninitialized MDP object!", path);
    return false;
  }

  try
  {
    // Read the file into C global variables.
    if(!readMDP((char*)path))
      return false;

    // Allocate memory.
    this->P_ = (Matrix *) XMALLOC( gNumActions * sizeof( *P ) );


    // Copy values over.
    for(int a = 0; a < gNumActions; a++)
    {
      copyMatrix(P[a], &(this->P_[a]));
    }
    state_cur_ = boost::make_shared<SimpleFeatureValue>(gInitialState);
    // Any aborts after this point will cause memory leaks if they don't deallocate P.

    // Initialize pretty printers. Currently won't fail if they fail.
    shared_ptr<Pretty> pretty_action = boost::make_shared<Pretty>();
    pretty_action->loadNames(path, "actions");

    shared_ptr<Pretty> pretty_state = boost::make_shared<Pretty>();
    pretty_state->loadNames(path, "states");

    // Initialize StateSpace.
    // @todo TODO Make this support all types of feature spaces! FeatureSpaceFactory?
    action_space_.reset(new SimpleFeatureSpace(gNumActions, pretty_action));
    ROS_INFO_STREAM("Added action space: " << *action_space_); // Not NULL because just initialized!

    state_space_.reset(new SimpleFeatureSpace(gNumStates, pretty_state));
    ROS_INFO_STREAM("Added state space: " << *state_space_); // Not NULL because just initialized!

    this-> is_init_ = true;
    return true;
  }
  catch(std::exception& err)
  {
    ROS_ERROR_STREAM("Error: " << err.what());
    return false;
  }

}

// Taken from mdp/mdp.h::void deallocateMDP()
MDP::~MDP()
{
  if(is_init_)
  {
    for( int a = 0; a < getNumActions(); a++ )
    {
      destroyMatrix( P[a] );
    }
    XFREE(P);
  }
}

/**
 * Finds an MDP within a <s>container</s> vector (for now) of MDP objects
 * by name field.
 *
 * @param mdps List of mdps to search.
 * @param name Name of desired mdp object.
 * @return Pointer to first mdp object found matching 'name'. NULL if not found.
 */
MDP* MDP::findMDP(const std::vector<MDP*>& mdps, std::string name)
{
  for(int i = 0; i < mdps.size(); i++)
  {
    if(mdps[i]->getName() == name)
      return mdps[i];
  }

  return NULL;
}

/**
 * Looks up a state transition probability.
 *
 * @param state_old Current state
 * @param state_new Anticipated state
 * @param action Action
 * @return Transition probability [0, 1] or -1 for failure.
 */
double MDP::lookupTransProb(const SimpleFeatureValue& state_old, const SimpleFeatureValue& state_new, int action)
{
  if(!isInit())
  {
    ROS_ERROR("Error: MDP not initialized.");
    return -1;
  }

  ROS_ASSERT_CMD((P_ != NULL), return -1);
  ROS_ASSERT_CMD(state_space_->isValid(state_old),
                 ROS_ERROR_STREAM("Invalid State: " << state_space_->toString(state_old)
                                  << "\n...for StateSpace: " << state_space_);
                 return -1);
  ROS_ASSERT_CMD(state_space_->isValid(state_new),
                 ROS_ERROR_STREAM("Invalid State: " << state_space_->toString(state_new)
                                  << "\n...for StateSpace: " << state_space_);
                 return -1);
  ROS_ASSERT_CMD(action_space_->isValid(SimpleFeatureValue(action)), return -1);

  return getEntryMatrix(this->P_[action], state_old.asInt(), state_new.asInt());
}

/**
 * Searches for actions that connect two states.
 *
 * @param state_old Current state.
 * @param state_new Anticipated state.
 * @param actions Vector of actions with nonzero probabilities.
 * @param trans_probs Probabilities corresponding to each action.
 * @return True for success, false for an error.
 */
// @todo Add an epsilon to define "close enough" to zero?
bool MDP::estimateAction(int state_old, int state_new, std::vector<int> &actions, std::vector<double> &trans_probs)
{
  ROS_ASSERT_CMD((P_ != NULL), return false);

  for(int a = 0; a < getNumActions(); a++)
  {
    double trans_prob = this->lookupTransProb(state_old, state_new, a);
    ROS_ASSERT_RNG_CMD(trans_prob, 0.0, 1.0, return false);
    if(trans_prob == -1)
      return false;
    else if(trans_prob > 0)
    {
      actions.push_back(a);
      trans_probs.push_back(trans_prob);
    }
  }

  return true;
}

/**
 * Updates fully-observed state.
 *
 * For MDPs, observation IS the new state.
 *
 * @param observation Observation number, in range [0, num_states).
 * @return True on success.
 */
bool MDP::updateBeliefState(int observation)
{
  ROS_ASSERT_CMD(state_space_->isValid(SimpleFeatureValue(observation)),
                 ROS_ERROR_STREAM("Invalid " << state_space_->toString(SimpleFeatureValue(observation))
                                  << "\n...for StateSpace: " << *state_space_);
                 return false);

  state_cur_ = boost::make_shared<SimpleFeatureValue>(observation);
  return true;
}


/**
 * Setter for associated policy.
 *
 * Checks that p is not empty and that policy and POMDP have the same number of
 * actions and states.
 *
 * @todo Some way to check actions (need to standardize enum parameters? Could have a valid policy lacking a
 * particular action.)
 *
 * @param p The policy.
 * @return True on success.
 */
bool MDP::setMyPolicy(shared_ptr<const Policy> p)
{
  ROS_ASSERT_MSG(p, "Policy pointer given to MDP '%s' is empty!", getName().c_str());
  ROS_ASSERT_CHILD_CMD(p, MDPPolicy, return false);
  if(p->getStateSpace() != this->getStateSpace())
  {
    ROS_ERROR("Policy '%s' and MDP '%s' have different state spaces:",
              p->getName().c_str(), getName().c_str());
    ROS_ERROR_STREAM(p->getStateSpace());
    ROS_ERROR("v.s.");
    ROS_ERROR_STREAM(getStateSpace());
    return false;
  }

  // Downcast is frowned upon, but more acceptable during initialization than during execution.
  my_policy_ = boost::dynamic_pointer_cast<const MDPPolicy>(p);
  return true;
}

/**
 * Applies associated policy.
 *
 * Uses internal state. Stores last applied action internally as well as returning it!
 *
 * @return Action or -1 for failure.
 */
int MDP::applyPolicy()
{
  ROS_ASSERT_MSG(my_policy_, "No Policy associated with MDP '%s'!", getName().c_str());

  if(!state_cur_)
    return -1;
  int a = my_policy_->applyPolicy(*state_cur_);
  if(a == -1)
    return -1;

  last_action_ = a;
  return last_action_;
}

int MDP::getNumActions() const
{
  return action_space_->getNumValues();
}

std::string MDP::prettyLastAction() const
{
  return prettyAction(last_action_);
}

std::string MDP::prettyAction(int action) const
{
  return action_space_->toString(SimpleFeatureValue(action));
}

std::string MDP::prettyObs(int observation) const
{
  return state_space_->toString(SimpleFeatureValue(observation));
}

bool MDP::setStateSpace(shared_ptr<const FeatureSpace> ss)
{
  // If new is NULL, there's no point.
  if(!ss)
  {
    ROS_ERROR("Empty StateSpace given!");
    return false;
  }

  // Perform this check only for existing StateSpace.
  if(state_space_)
    if(ss->getNumValues() != this->state_space_->getNumValues())
    {
      ROS_ERROR("Trying to replace StateSpace with incompatible one!");
      // Already checked for NULL.
      ROS_ERROR_STREAM("Existing StateSpace: " << *state_space_);
      ROS_ERROR_STREAM("New StateSpace: " << *ss);
      return false;
    }

  // Either old is NULL or new has same number of states. Replace it.
  state_space_ = ss;
  return true;
}

std::string MDP::prettyState() const
{
  if(!state_cur_)
    return "NULL";

  return prettyState(*state_cur_);
}


std::string MDP::prettyState(const FeatureValue& state) const
{
  return state_space_->toString(state);
}

/**
 * Makes a hard copy of an entire transition matrix.
 *
 * @param a Action requested.
 * @return Transition matrix or NULL on failure. Needs to be free()d.
 */
Matrix MDP::getTransMatrix(int a)
{
  ROS_ASSERT_CMD(isInit(), return NULL);
  ROS_ASSERT_CMD((action_space_->isValid(SimpleFeatureValue(a))), return NULL);
  ROS_ASSERT_CMD((P_ != NULL), return NULL);

  Matrix ret;
  copyMatrix(P_[a], &ret);
  return ret;
}

} /* namespace pomdp */
