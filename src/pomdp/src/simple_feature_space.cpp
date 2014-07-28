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
#include <XmlRpcValue.h>

#include "assert_rng.h"
#include "belief_feature_value.h"
#include "feature_value.h"
#include "simple_feature_space.h"
#include "simple_feature_value.h"

namespace pomdp
{

/**
 * A constant to avoid magic numbers.
 *
 * Bland name because there's only a single feature.
 */
const std::string DEFAULT_NAME = "feature space";

SimpleFeatureSpace::SimpleFeatureSpace()
{
 SimpleFeatureSpace(0);
}

SimpleFeatureSpace::SimpleFeatureSpace(unsigned int num_states, const std::vector<std::string>& name_mapping)
{
  SimpleFeatureSpace(num_states, boost::make_shared<Pretty>(name_mapping));

  if(feature_.num_states_ >= 2)
    is_init_ = true;
}

SimpleFeatureSpace::SimpleFeatureSpace(unsigned int num_states, shared_ptr<const Pretty> name_mapping)
{
  feature_.name_ = DEFAULT_NAME;
  feature_.num_states_ = num_states;
  feature_.state_name_mapping_ = name_mapping;

  if(feature_.num_states_ >= 2)
    is_init_ = true;
}

SimpleFeatureSpace::~SimpleFeatureSpace()
{
}

bool SimpleFeatureSpace::loadFromYAML(const YAML::Node& feature_space_root)
{
  try
  {
    // Read name
    feature_space_root >> feature_;

    // Sanity checks.
    if((int)feature_.num_states_ < 0) // Negative number of states
    {
      ROS_ERROR_STREAM("Negative number of states " << (int)feature_.num_states_ << " in SimpleFeatureSpace '"
                       << feature_.name_ << "'");
      return false;
    }
    if(feature_.num_states_ < 2)
    {
      ROS_ERROR_STREAM("Number of states " << (int)feature_.num_states_ << " in SimpleFeatureSpace '" << feature_.name_
                       << "' too short for meaningful state space.");
      return false;
    }
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

bool SimpleFeatureSpace::isValid(const FeatureValue& state) const
{
  if(typeid(state) == typeid(SimpleFeatureValue))
    return isValid(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    return isValid(dynamic_cast<const BeliefFeatureValue&>(state));
  else
    return false;
}

bool SimpleFeatureSpace::isValid(const SimpleFeatureValue& state) const
{
  return state.asInt() < getNumValues();
}

bool SimpleFeatureSpace::isValid(const BeliefFeatureValue& belief) const
{
  return belief.asVector().size() == getNumValues();
}

std::string SimpleFeatureSpace::toString() const
{
  std::stringstream ret;
  ret << "SimpleFeatureSpace, has " << feature_;
  return std::string(ret.str());
}

/**
 * Converts a FeatureValue into a name (MDPStates) or a list of probabilities (POMDPStates).
 *
 * @param s The state.
 * @return Stringified State.
 */
std::string SimpleFeatureSpace::toString(const FeatureValue& state) const
{
  std::stringstream ret;
  ret << boost::units::detail::demangle(typeid(state).name());
  if(typeid(state) == typeid(SimpleFeatureValue))
    ret << ": " << toString(dynamic_cast<const SimpleFeatureValue&>(state));
  else if(typeid(state) == typeid(BeliefFeatureValue))
    ret << ": " << toString(dynamic_cast<const BeliefFeatureValue&>(state));
  return ret.str();
}

std::string SimpleFeatureSpace::toString(const SimpleFeatureValue& state) const
{
  return feature_.state_name_mapping_->toString(state.asInt());
}

std::string SimpleFeatureSpace::toString(const BeliefFeatureValue& belief) const
{
  return Pretty::vectorToString(belief.asVector());
}

/**
 * Reads a feature value from a string.
 *
 * String must be either a name contained in this state space (e.g. "go_left") or a number within range (e.g. 4).
 *
 * @param str String representation of the value.
 * @return A SimpleFeatureValue. Value is invalid on failure.
 */
shared_ptr<FeatureValue> SimpleFeatureSpace::readFeatureValue(const std::string& str) const
{
  const unsigned int INVALID_VALUE = INT_MAX;

  int value = Pretty::readNameOrNum(str, feature_.state_name_mapping_);
  if(value == -1)
    return boost::make_shared<SimpleFeatureValue>(INVALID_VALUE);
  else
    return boost::make_shared<SimpleFeatureValue>(value); // Doesn't check for validity - that will be done when isValid is called.
}

void SimpleFeatureSpace::toXmlRpcValue(XmlRpc::XmlRpcValue& v) const
{
  using namespace XmlRpc;

  // Gets value names if they exist or stringified numbers if they don't.
  std::vector<std::string> value_names;
  for(unsigned int i = 0; i < feature_.num_states_; i++)
  {
    value_names.push_back(feature_.state_name_mapping_->toString(i));
  }

  //
  // Create child nodes.
  //
  XmlRpcValue name(feature_.name_);

  // xmlRpcValue(std::vector) would go here
  XmlRpcValue values;
  values.setSize(value_names.size()); // Converts xmlRpcValue to a vector type.

  // Copy the contents into the XmlRpcValue
  for(size_t i=0; i < value_names.size(); i++) {
    values[i] = value_names.at(i);
  }
  
  //
  // Add children to parent node.
  //
  v["name"] = name;
  v["values"] = values;
}


} /* namespace pomdp */
