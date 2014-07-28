/**
 * @file
 *
 *
 *
 * @date Aug 22, 2013
 * @author Bea Adkins
 */
#include "feature.h"

namespace pomdp
{

/**
 * Reads YAML node into a Feature. Throws YAML::Exception.
 *
 * Example YAML node:
 * @code{.yaml}
 * name: affect
 * states: neutral happy sad angry afraid
 * @endcode
 *
 * Example usage:
 * @code{.cpp}
 * Feature f;
 * YAML::Node& feature_vector_root = doc["features"]; // doc read above.
 * feature_vector_root >> f;
 * @endcode
 *
 * @todo Refactor to Feature/StateSpace I/O class.
 * @param[in] Properly formed YAML node.
 * @param[out] f Initialized Feature.
 */
void operator >> (const YAML::Node& node, Feature& f)
{
  // Scratch variable
  std::vector<std::string> state_names;

  // Read name
  node["name"] >> f.name_;

  // Read and parse states
  switch(node["values"].Type())
  {
    case YAML::NodeType::Scalar:
    {
      // Read as integer number of states.
      node["values"] >> f.num_states_;
      f.state_name_mapping_ = Pretty::empty;
      break;
    }
    case YAML::NodeType::Sequence:
    {
      // Read as sequence of state names.
      node["values"] >> state_names;
      f.state_name_mapping_.reset(new Pretty(state_names));
      f.num_states_ = state_names.size();
      break;
    }
    default:
      return;
  }
}

} /* namespace pomdp */
