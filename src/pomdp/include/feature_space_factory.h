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
