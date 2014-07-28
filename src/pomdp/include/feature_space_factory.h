/**
 * @file
 *
 * Factory for creating FeatureSpace objects from YAML configurations.
 *
 * @date Nov 15, 2013
 * @author Bea Adkins
 */

#ifndef FEATURE_SPACE_FACTORY_H
#define FEATURE_SPACE_FACTORY_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include "feature_space.h"

using boost::shared_ptr; // Because it's long and ugly enough v.s. a '*'.

namespace pomdp
{

class FeatureSpaceFactory
{
public:
  FeatureSpaceFactory(){};
  virtual ~FeatureSpaceFactory(){};

  /**
   * Loads a FeatureSpace from a YAML config file.
   *
   * @param path Path to file.
   * @param key Name of root node, e.g. "state_space".
   * @return Smart pointer to initialized FeatureSpace object. Empty (boolean value of false) on failure.
   */
  shared_ptr<FeatureSpace> loadFromFile(const std::string& path, const std::string& key);

  /**
   * Loads a FeatureSpace from a YAML node.
   *
   * @param feature_space_root Root node, e.g. "state_space".
   * @return Smart pointer to initialized FeatureSpace object. Empty (boolean value of false) on failure.
   */
  shared_ptr<FeatureSpace> loadFromYAML(const YAML::Node& feature_space_root);
};

} /* namespace pomdp */
#endif /* FEATURE_SPACE_FACTORY_H */
