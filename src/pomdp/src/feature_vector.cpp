/**
 * @file
 *
 *
 *
 * @date Aug 20, 2013
 * @author Bea Adkins
 */

#include <fstream>
#include <iostream>

#include "feature_vector.h"

namespace pomdp
{
/**
 * Reads YAML node into a Feature. Throws YAML::Exception.
 *
 * Example YAML node:
 * @code{.yaml}
 *   - name: visible
 *     states: true false   # Can list state names, from which numbers will be counted...
 *   - name: color
 *     states: red blue green yellow purple
 *   - name: x
 *     states: 9            # ... or can just supply number of states.
 *   - name: y
 *     states: 9
 * @endcode
 *
 * Example usage:
 * @code{.cpp}
 * FeatureVector fv;
 * YAML::Node& root; // Read somewhere above.
 * root >> fv;
 * @endcode
 *
 * @todo Refactor to Feature/StateSpace I/O class.
 * @param[in] Properly formed YAML node.
 * @param[out] fv Initialized FeatureVector.
 */
void operator>>(const YAML::Node& node, FeatureVector& fv)
{
  for(YAML::Iterator itr = node.begin(); itr != node.end(); itr++)
  {
    // Read feature, store if valid.
    Feature feature_cur;
    *itr >> feature_cur;

    if(feature_cur.valid())
      fv.addFeature(feature_cur);
  }
}

/**
 * Parses YAML config files.
 *
 * Example file:
 * @code{.yaml}
 * features:
 *   - name: visible
 *     states: true false   # Can list state names, from which numbers will be counted...
 *   - name: color
 *     states: red blue green yellow purple
 *   - name: x
 *     states: 9            # ... or can just supply number of states.
 *   - name: y
 *     states: 9
 * @endcode
 *
 * @param Path to file.
 * @return Loaded FeatureVector, or FeatureVector::empty on failure.
 */
FeatureVector FeatureVectorFactory::loadFromFile(const char* path)
{
  // Open the file
  std::ifstream feature_file(path);
  if(!feature_file.good())
  {
    ROS_ERROR_STREAM("Unable to open feature file '" << path << "'.");
    return FeatureVector::empty;
  }

  try
  {
    YAML::Parser features_parser(feature_file);

    YAML::Node doc;
    if(!features_parser.GetNextDocument(doc))
    {
      ROS_ERROR("Unable to parse file's root YAML Node.");
      return FeatureVector::empty;
    }

    return loadFromYAML(doc);
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return FeatureVector::empty;
  }
}

FeatureVector FeatureVectorFactory::loadFromYAML(const YAML::Node& feature_space_root)
{
  try
  {
    // Read a feature vector from the root node.
    FeatureVector ret;
    feature_space_root >> ret;
    return ret;
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return FeatureVector::empty;
  }
}

//static
const FeatureVector& FeatureVector::empty = FeatureVector(std::vector<Feature>());

} /* namespace pomdp */
