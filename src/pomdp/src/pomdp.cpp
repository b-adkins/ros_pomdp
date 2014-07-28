#include "pomdp.h"

#include "assert_type.h"
#include "belief_feature_value.h"
#include "feature_value.h"
#include "pretty.h"
#include "feature_space.h"

namespace pomdp
{

/**
 * Does not allocate any of the POMDP variables!
 */
POMDP::POMDP() : num_observations_(0), R_(NULL), my_policy_(), pretty_observation_(new Pretty())
{
  //4noobs MDP() implicitly called because it's parent class.
  state_cur_ = boost::make_shared<BeliefFeatureValue>();
}

bool POMDP::loadFromFile(const char* path)
{
  if(!MDP::loadFromFile(path))
    return false;
  is_init_ = false; // For a POMDP, not finished yet!

  // Check C problem type flag to ensure that this is a POMDP that was loaded.
  if(gProblemType != POMDP_problem_type)
    return false;
  
  try
  {
    // Allocate memory.
    this->R_ = (Matrix *) XMALLOC( gNumActions * sizeof( *R ) );

    // Copy values over.
    this->num_observations_ = gNumObservations;
    for(int a = 0; a < getNumActions(); a++)
    {
      copyMatrix(R[a], &(this->R_[a]));
    }
    // Any aborts after this point will cause memory leaks if they don't deallocate R.

    std::vector<double> initial_state(gInitialBelief, gInitialBelief + gNumStates);
    state_cur_ = boost::make_shared<BeliefFeatureValue>(initial_state);

    // Initialize pretty printers. Currently won't fail if they fail.
    boost::const_pointer_cast<Pretty>(pretty_observation_)->loadNames(path, "observations");
    ROS_DEBUG_STREAM("Added pretty printer for observations: " << *pretty_observation_);
  }
  catch(std::exception& err)
  {
    ROS_ERROR_STREAM("Error: " << err.what());
    return false;
  }

  is_init_ = true;
  return true;
}

// Taken from mdp/mdp.h::void deallocateMDP()
POMDP::~POMDP()
{
  if(is_init_)
  {
    for( int a = 0; a < action_space_->getNumValues(); a++ )
    {
      destroyMatrix( R[a] );
    }
    XFREE(R);
  }
}

/**
 * Using the internal observation model and stored last action, updates
 * internal belief state. 
 *
 * Preferred method if using stored last action. 
 *
 * Not thread safe! (Manipulates underlying C global variables.)
 *
 * @param observation State observed.
 * @return True on success.
 */
bool POMDP::updateBeliefState(int observation)
{
  return updateBeliefState(observation, last_action_);
}

/**
 * Using the internal observation model, updates internal belief state.
 *
 * Not remotely thread safe! (Manipulates underlying C global variables.)
 *
 * @param observation Observation number, in range [0, num_observations).
 * @param action Last action taken, in range [0, num_actions).
 * @return True on success.
 */
// Wraps mdp/mdp.h::int transformBeliefState()
bool POMDP::updateBeliefState(int observation, int action)
{
  if(!isInit())
  {
    ROS_ERROR("Error: POMDP not initialized.");
    return false;
  }

  ROS_ASSERT_CMD((action_space_->isValid(SimpleFeatureValue(action))), return false);
  ROS_ASSERT_INDEX_CMD(observation, 0, num_observations_, return false;);

  // Return vector for new belief.
  // @todo TODO Optimization: allocate this memory within the class instead of on the function stack?
  std::vector<double> belief_tmp(state_space_->getNumValues());
  double* belief_tmp_c = &belief_tmp[0]; // Valid as long as belief_tmp doesn't reallocate.

  // Kludgy variable shuffle! Saves global values/pointers...
  Matrix* P_global = P;
  Matrix* R_global = R;
  Problem_Type gProblemType_global = gProblemType;
  int gNumStates_global = gNumStates;

  // ...replaces with class member values/pointers...
  P = this->P_;
  R = this->R_;
  gProblemType = POMDP_problem_type;
  gNumStates = state_space_->getNumValues();

  shared_ptr<BeliefFeatureValue> belief_state = boost::dynamic_pointer_cast<BeliefFeatureValue>(state_cur_);
  // transformBeliefState() doesn't change field pi, so const_cast smells but works.
  bool success = (bool)transformBeliefState(const_cast<double*>(belief_state->asArray()), belief_tmp_c,
                                            action, observation);
  if(success)
  {
    // Assign returned belief to POMDP's belief.
    state_cur_ = boost::make_shared<BeliefFeatureValue>(belief_tmp);
  }
  else
  {
    ROS_WARN("Observation suggests impossible state transition for %s. (a = %s, z = %s.)",
             getName().c_str(), prettyAction(last_action_).c_str(), prettyObs(observation).c_str());
  }

  // ...and puts the globals back.
  P = P_global;
  R = R_global;
  gProblemType = gProblemType_global;
  gNumStates = gNumStates_global;

  // Save last action
  last_action_ = action;

  return success;
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
bool POMDP::setMyPolicy(shared_ptr<const Policy> p)
{
  ROS_ASSERT_MSG(p, "Policy pointer given to POMDP '%s' is empty!", getName().c_str());
  ROS_ASSERT_CHILD_CMD(p, POMDPPolicy, return false);

  if(p->getStateSpace() != this->getStateSpace())
  {
    ROS_ERROR("Policy '%s' and POMDP '%s' have different state spaces:",
              p->getName().c_str(), getName().c_str());
    ROS_ERROR_STREAM(p->getStateSpace());
    ROS_ERROR("v.s.");
    ROS_ERROR_STREAM(getStateSpace());
    return false;
  }

  // Downcast is frowned upon, but more acceptable during initialization than during execution.
  my_policy_ = boost::dynamic_pointer_cast<const POMDPPolicy>(p);
  return true;
}

/**
 * Applies associated policy.
 *
 * Uses internal state. Stores last applied action internally as well as returning it!
 *
 * @return Action or -1 for failure.
 */
int POMDP::applyPolicy()
{
  ROS_ASSERT_MSG(my_policy_, "No Policy associated with POMDP '%s'!", getName().c_str());

  int a = my_policy_->applyPolicy(*state_cur_);
  if(a == -1)
    return false;

  last_action_ = a;
  return last_action_;
}

std::string POMDP::prettyObs(int observation)
{
  return pretty_observation_->toString(observation);
}

} /* namespace pomdp */
