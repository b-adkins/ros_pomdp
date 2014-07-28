/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Boone "Bea" Adkins
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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
