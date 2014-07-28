/**
 * @file
 *
 * Class for creating FeatureSpaces.
 *
 * @date Nov 15, 2013
 * @author Bea Adkins
 */

#include <iostream>
#include <fstream>

#include "feature_space_factory.h"

#include "simple_feature_space.h"
#include "vector_feature_space.h"

namespace pomdp
{

shared_ptr<FeatureSpace> FeatureSpaceFactory::loadFromFile(const std::string& path, const std::string& key)
{
  std::ifstream file;
  file.open(path.c_str());

  try
  {
    YAML::Parser parser(file);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    return loadFromYAML(doc[key]);
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return shared_ptr<FeatureSpace>();
  }
}

shared_ptr<FeatureSpace> FeatureSpaceFactory::loadFromYAML(const YAML::Node& feature_space_root)
{
  shared_ptr<FeatureSpace> ret;
  if(feature_space_root.Type() == YAML::NodeType::Map)
  {
    ret.reset(new SimpleFeatureSpace());
      if(ret->loadFromYAML(feature_space_root))
        return ret;
  }
  else if(feature_space_root.Type() == YAML::NodeType::Sequence)
  {
    ret.reset(new VectorFeatureSpace());
    if(ret->loadFromYAML(feature_space_root))
        return ret;
  }

  // Otherwise fail.
  return shared_ptr<FeatureSpace>();
}

} /* namespace pomdp */
