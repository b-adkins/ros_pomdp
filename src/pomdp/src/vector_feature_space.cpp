/**
 * @file
 *
 *
 *
 * @date Aug 22, 2013
 * @author Bea Adkins
 */

#include <boost/units/detail/utility.hpp>
#include <sstream>

#include "vector_feature_space.h"

#include "belief_feature_value.h"
#include "simple_feature_value.h"

namespace pomdp
{
/** Create from FeatureVector file. Will be empty (getNumStates() == 0) on failure. */
VectorFeatureSpace::VectorFeatureSpace(const char* path)
{
  FeatureVectorFactory factory;
  feature_vector_ = factory.loadFromFile(path);
  if(feature_vector_.size() > 0)
  //  if(feature_vector_ != FeatureVector::empty) // Success
    is_init_ = true;
}

bool VectorFeatureSpace::loadFromYAML(const YAML::Node& feature_space_root)
{
  FeatureVectorFactory fact;
  feature_vector_ = fact.loadFromYAML(feature_space_root);
  is_init_ = feature_vector_ != FeatureVector::empty;
  return is_init_;
}

bool VectorFeatureSpace::isValid(const FeatureValue& state) const
{
  if(typeid(state) == typeid(SimpleFeatureValue))
    return isValid(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    return isValid(dynamic_cast<const BeliefFeatureValue&>(state));
  else if(typeid(state) == typeid(FeatureVectorValue))
    return isValid(dynamic_cast<const FeatureVectorValue&>(state));
  else
    return false;
}

bool VectorFeatureSpace::isValid(const SimpleFeatureValue& state) const
{
  return state.asInt() < getNumValues();
}

bool VectorFeatureSpace::isValid(const BeliefFeatureValue& belief) const
{
  return belief.asVector().size() == getNumValues();
}

bool VectorFeatureSpace::isValid(const FeatureVectorValue& state) const
{
  return feature_vector_.validValue(state);
}

FeatureValue VectorFeatureSpace::hash(const FeatureValue& state) const
{
  if(typeid(state) == typeid(SimpleFeatureValue))
    return hash(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    return hash(dynamic_cast<const BeliefFeatureValue&>(state));
  else //if(typeid(state) == typeid(FeatureVectorValue))
    // @todo Make this not hideous
    return hash(dynamic_cast<const FeatureVectorValue&>(state));
}

SimpleFeatureValue VectorFeatureSpace::hash(const SimpleFeatureValue& state) const
{
  return SimpleFeatureValue(state);
}

BeliefFeatureValue VectorFeatureSpace::hash(const BeliefFeatureValue& belief) const
{
  throw std::runtime_error("VectorFeatureSpaces do not yet support hashing of BeliefStates.");
}

SimpleFeatureValue VectorFeatureSpace::hash(const FeatureVectorValue& state) const
{
  return SimpleFeatureValue(feature_vector_.toStateNumber(state));
}

FeatureValue VectorFeatureSpace::dehash(const FeatureValue& state) const
{
  if(typeid(state) == typeid(SimpleFeatureValue))
    return dehash(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    return dehash(dynamic_cast<const BeliefFeatureValue&>(state));
  else //if(typeid(state) == typeid(FeatureVectorValue))
    // @todo Make this not hideous
    return dehash(dynamic_cast<const FeatureVectorValue&>(state));
}

FeatureVectorValue VectorFeatureSpace::dehash(const SimpleFeatureValue& state) const
{
  return feature_vector_.toFeatureVectorValue(state.asInt());
}

BeliefFeatureValue VectorFeatureSpace::dehash(const BeliefFeatureValue& belief) const
{
  throw std::runtime_error("VectorFeatureSpaces do not yet support dehashing of BeliefStates.");
}

FeatureVectorValue VectorFeatureSpace::dehash(const FeatureVectorValue& state) const
{
  return state;
}

std::string VectorFeatureSpace::toString() const
{
  std::stringstream ret;
  ret << "VectorStateSpace, size " << getNumValues() << ", has " << feature_vector_;
  return std::string(ret.str());
}

// @todo Refactor to FeatureVectorValue or FeatureVector
std::string VectorFeatureSpace::toString(const FeatureValue& state) const
{
  std::stringstream ret;
  ret << boost::units::detail::demangle(typeid(state).name());
  if(typeid(state) == typeid(SimpleFeatureValue))
    ret << ": " << toString(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    ret << ": " << toString(dynamic_cast<const BeliefFeatureValue&>(state));
  else if(typeid(state) == typeid(FeatureVectorValue))
    ret << ": " << toString(dynamic_cast<const FeatureVectorValue&>(state));

  return ret.str();
}

std::string VectorFeatureSpace::toString(const SimpleFeatureValue& state) const
{
  return toString(feature_vector_.toFeatureVectorValue(state.asInt()));
}

/**
 * Same as SimpleStateSpace::toString(const BeliefState&). Need to find a better way.
 */
std::string VectorFeatureSpace::toString(const BeliefFeatureValue& belief) const
{
  return Pretty::vectorToString(belief.asVector());
}


std::string VectorFeatureSpace::toString(const FeatureVectorValue& state) const
{
  std::stringstream ret;

  // Print each Feature's value.
  ret << "[ ";
  for(int i = 0; i < feature_vector_.size(); i++)
  {
    Feature f = feature_vector_[i];
    ret << f.name_ << ": " << f.state_name_mapping_->toString(state[i]) << ", ";
  }
  ret << "]";

  return ret.str();
}

/**
 * Reads a feature vector value from a string.
 *
 * Values must be either a name (string) contained in this state space (e.g. "go_left") or a number within range
 * (e.g. 4). String elements should be delimited by spaces. @todo Or commas.
 *
 * @param str String representation of the value.
 * @return A FeatureVectorValue. Value is invalid on failure.
 */
shared_ptr<FeatureValue> VectorFeatureSpace::readFeatureValue(const std::string& str) const
{
  const std::vector<unsigned int> INVALID_VALUE;

  std::vector<unsigned int> feature_values;
  std::stringstream str_ss(str);
  std::string word;
  for(int i = 0; i < feature_vector_.size(); i++)
  {
    if(!(str_ss >> word))
    {
      ROS_ERROR_STREAM("Insufficient values in line '" << str << "' to read a FeatureVectorValue in space " << *this);
      return boost::make_shared<FeatureVectorValue>(INVALID_VALUE);
    }

    int value = Pretty::readNameOrNum(str, feature_vector_[i].state_name_mapping_);
    if(value == -1)
    {
      ROS_ERROR_STREAM("Invalid value '" << word << "'.");
      return boost::make_shared<FeatureVectorValue>(INVALID_VALUE);
    }
    else
      feature_values.push_back(value);
  }

  return boost::make_shared<FeatureVectorValue>(feature_values); // Doesn't check for validity - that will be done when isValid is called.
}

}
/* namespace pomdp */
