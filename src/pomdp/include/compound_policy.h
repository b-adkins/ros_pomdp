/**
 * @class pomdp::CompoundPolicy
 *
 * Common interface for any policy composed of a number of other policies.
 *
 * Makes possible polymorphism between CompoundPolicy variants with diverse sub-policies.
 *
 * The class template CompoundPolicySpec can be used to actually implement a CompoundPolicy. It provides
 * several methods to help build a class. Sub-classes of the abstract classes ::CompoundMDPPolicy and
 * ::CompoundPOMDPPolicy should be used with MDPs and POMDPs, respectively.
 *
 * @date Aug 9, 2013
 * @author Bea Adkins
 *
 * @class pomdp::CompoundPolicySpec< Base_Policy >
 *
 * Abstract class template for families of CompoundPolicy classes.
 *
 * File format is simply a list of component policy config files.
 * @code
 * totally_sure.mdp_policy
 * hedges_options.alpha_policy
 * @endcode
 *
 * To avoid absolute paths, cfg_root in the <a href="http://wiki.ros.org/Parameter%20Server">param server</a> should be
 * set to the root directory of the listed files, and the node should concatenate it to the relative paths.
 * @code{.sh}
   $ rosparam set cfg_root "path/to/config"
   @endcode
 * @code{.xml}
   <param name="cfg_root" value="$(find pomdp_node)/launch/this_node" />
   @endcode
 *
 * @todo Refactor model weights to WeightedCompoundPolicy, and the stochastic elements (all the model likelihoods) to a
 * subclass, allowing this to also be superclass for any deterministic CompoundPolicy.
 *
 * @date Aug 9, 2013
 * @author Bea Adkins
 */

#ifndef COMPOUND_POLICY_H
#define COMPOUND_POLICY_H

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <fstream>
#include <iostream>
#include <numeric>

#include "assert_math.h"
#include "policy_factory.h"
#include "pomdp_io.h"

#include "mdp_policy.h"
#include "pomdp_policy.h"

namespace pomdp
{

class CompoundPolicy : public virtual Policy
{
public:
  CompoundPolicy(){};
  virtual ~CompoundPolicy(){};
  virtual bool loadFromFiles(const std::vector<std::string>& paths) = 0;

  /**
   * Variations of this method, returning other data structures, may be useful to implement in child classes.
   */
  virtual void applyPolicies(const FeatureValue& current, std::vector<int>& actions) const = 0;
  virtual int getNumModels() const = 0;
  virtual void getModelNames(std::vector<std::string>& model_names) const = 0;

  // @todo TODO Setters, maybe getters, for individual model attributes 
  // E.g. setAttr(index, value), setAttr(name, value)

  // @todo TODO Refactor to WeightedCompoundPolicy
  virtual void getModelLikelihoods(std::vector<double>& model_likelihoods) const  = 0;
  virtual bool setModelLikelihoods(const std::vector<double>& model_likelihoods)  = 0;
  virtual bool resetModelLikelihoods() = 0;
  virtual bool normalizeModelLikelihoods() = 0;
};

/**
 * @tparam Base_Policy A Policy. Each object "has a" list of Base_Policies (e.g. MDPPolicy or
 * POMDPPolicy) and "is a" Base_Policy.
 */
template <class Base_Policy>
class CompoundPolicySpec : public virtual Base_Policy, public CompoundPolicy
{
private:
  BOOST_STATIC_ASSERT_MSG((boost::is_base_and_derived<Policy,Base_Policy>::value),
                          "CompoundPolicy templates must be instantiated with a Policy!");

public:
  CompoundPolicySpec(){};
  virtual ~CompoundPolicySpec(){};

  virtual bool loadFromFile(const char* path);
  virtual bool loadFromFiles(const std::vector<std::string>& paths);

  using Policy::applyPolicy; // Keeps this method pure virtual and this class abstract

  /**
   * Runs policies contained within. They are not combined in any way.
   *
   * @param[in] current Current state.
   * @param[out] actions Action or -1 for failure, one for each policy.
   */
  virtual void applyPolicies(const FeatureValue& current, std::vector<int>& actions) const
  {
    actions.clear();
    actions.resize(models_.size());
    if(!CompoundPolicySpec<Base_Policy>::isInit())
    {
      std::fill(actions.begin(), actions.end(), -1);
      return;
    }

    int p = 0;
    BOOST_FOREACH(shared_ptr<Base_Policy> policy, models_)
    {
      actions[p] = policy->applyPolicy(current);
      p++;
    }
  }


  virtual void getModelNames(std::vector<std::string>& model_names) const
  {
    model_names.clear();
    BOOST_FOREACH(shared_ptr<Base_Policy> policy, models_)
    {
      model_names.push_back(policy->getName());
    }
  }

  virtual void getModelLikelihoods(std::vector<double>& model_likelihoods) const
  {
    model_likelihoods.clear();
    model_likelihoods.assign(this->model_likelihoods_.begin(), this->model_likelihoods_.end());
  }

  /**
   * Checks that vector contains valid probabilities. (They add up to 1.)
   *
   * @return True on success.
   */
  virtual bool setModelLikelihoods(const std::vector<double>& model_likelihoods)
  {
    // Check for valid probability.
    ROS_ASSERT_VALID_PMF_CMD(model_likelihoods, 1e-5, return false);

    this->model_likelihoods_ = model_likelihoods;
    return true;
  }

  /**
   * Sets model likelihoods to a uniform distribution.
   *
   * @return True on success.
   */
  virtual bool resetModelLikelihoods()
  {
    // Prevent divide by zero.
    if(models_.size() == 0)
      return false;

    model_likelihoods_ = std::vector<double>(models_.size(), 1.0/models_.size());
    return true;
  }

  /**
   * Normalizes model likelihoods to add up to 1.0.
   *
   * @return True on success.
   */
  virtual bool normalizeModelLikelihoods()
  {
    // Add elements.
    double sum = std::accumulate(model_likelihoods_.begin(), model_likelihoods_.end(), 0.0);
    // Prevent divide by zero.
    if(sum == 0)
      return false;

    // Normalize.
    for(int i = 0; i < model_likelihoods_.size(); i++)
      model_likelihoods_[i] /= sum;
    return true;
  }

  virtual int getNumModels() const{ return model_likelihoods_.size(); }
  virtual int getNumStates() const
  {
    if(models_.size() == 0)
      return 0;

    // @todo TODO assert(num states equal for all policies)

    return models_[0]->getNumStates();
  }

  // Will uncomment when this has been tested.
  /**
   * Adds a Policy to the CompoundPolicy.
   *
   * Model likelihood for policy starts at zero.
   *
   * @policy The Policy.
   */
//  virtual bool addModel(shared_ptr<Base_Policy> policy)
//  {
//    if(!policy)
//      return false;
//
//    models_.push_back(policy);
//    // If this is the first model, set its likelihood to unity to satisfy sum(P(m)) = 1. Otherwise, use zero to not
//    // disturb other P(m)s.
//    model_likelihoods_.push_back(models_.size() == 1 ? 1.0 : 0.0);
//
//    // If it wasn't initialized, now that it has at least 1 models, it is.
//    if(!Base_Policy::isInit())
//      Base_Policy::is_init_ = true;
//
//    return true;
//  }

  /**
   * Removes a Policy from the CompoundPolicy.
   *
   * Model likelihoods must be renormalized.
   */
//  virtual bool removeModel(shared_ptr<Base_Policy> policy)
//  {
//  if(!policy)
//    return false;
//
//    typename std::vector<shared_ptr<Base_Policy> >::iterator to_remove = std::find(models_.begin(), models_.end(), policy);
//
//    // Not found.
//    if(to_remove == models_end())
//      return false;
//
//    // Found, remove.
//    models_.erase(to_remove);
//
//    // If it drops to 0 models, it's no longer initialized.
//    if(models_.size() == 0)
//      is_init_ = false;
//
//    return true;
//  }
protected:
  std::vector<double> model_likelihoods_;
  std::vector<shared_ptr<Base_Policy> > models_;
};

/**
 * Loads CompundPolicy from a file.
 *
 * File format is a list of paths to component policy config files, one per line.
 *
 * @tparam Base_Policy Every policy loaded must be a BasePolicy to succeed.
 * @param path Path to index file.
 * @return True on success.
 */
template <class Base_Policy>
bool CompoundPolicySpec<Base_Policy>::loadFromFile(const char* path)
{
  std::vector<std::string> paths;

  // Open file at paths.
  std::ifstream index_file;
  index_file.open(path);
  if(!index_file.good())
  {
    ROS_ERROR_STREAM("Unable to open file " << path);
    return false;
  }

  // Get root directory of config file.
  namespace bfs = boost::filesystem;
  bfs::path dir = bfs::path(path).parent_path();

  // Read paths from each line.
  std::string line;
  while(!index_file.eof())
  {
    std::getline(index_file, line);
    if(index_file.fail())
      continue;

    line = stripComment(line);
    if(isBlank(line))
      continue;

    // Add root directory
    bfs::path policy_path = dir / bfs::path(line);

    // Store
    paths.push_back(policy_path.string());
  }

  return loadFromFiles(paths);
}

template <class Base_Policy>
bool CompoundPolicySpec<Base_Policy>::loadFromFiles(const std::vector<std::string>& paths)
{
  ROS_ASSERT_MSG(paths.size() >= 2, "A CompoundPolicy requires at least two component policies.");
  ROS_ASSERT_MSG(state_space_, "Requires StateSpace to be initialized!");

  // Construct each policy.
  for(int p = 0; p < paths.size(); p++)
  {
    // Will fail without valid state space or if sub-policies have different number of states.
    shared_ptr<Base_Policy> new_policy = PolicyFactory::loadFromFile<Base_Policy>(paths[p], state_space_, action_space_);
    ROS_ASSERT_MSG(new_policy, "%s not a valid policy file!", paths[p].c_str());
    models_.push_back(new_policy);

    // Models are checked for consistent size due to common StateSpace.
  }

  // Initialize model likelihoods to uniform distribution.
  // @todo TODO Refactor to WeightedCompoundPolicy
  resetModelLikelihoods();

  // Default name
  setName("compound policy");

  is_init_ = true;
  return true;
}

typedef CompoundPolicySpec<MDPPolicy> CompoundMDPPolicy;
typedef CompoundPolicySpec<POMDPPolicy> CompoundPOMDPPolicy;

} /* namespace pomdp */
#endif /* COMPOUND_POLICY_H */
