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
