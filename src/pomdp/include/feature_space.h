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
 * @class pomdp::FeatureSpace
 *
 * Abstract class that describes a Feature space - state, action, observation, etc.
 *
 * @date Aug 22, 2013
 * @author Bea Adkins
 */

#ifndef FEATURE_SPACE_H
#define FEATURE_SPACE_H

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "pretty.h"

namespace pomdp
{
const unsigned int CONTINUOUS = INT_MAX;

// No need to tangle dependencies.
class FeatureValue;

class FeatureSpace
{
public:
  FeatureSpace() : is_init_(false){};
  virtual ~FeatureSpace(){};

  /**
   * Reads from a YAML tree (created from either file or string).
   *
   * @param feature_space_root Node representing "state_space", "action_space", etc.
   * @return True on success.
   */
  virtual bool loadFromYAML(const YAML::Node& feature_space_root) = 0;

  /**
   * Checks if a State is of type and value that lie within this state space.
   *
   * @return True if valid.
   */
  virtual bool isValid(const FeatureValue& state) const = 0;

  /**
   * Converts this object to a string.
   */
  virtual std::string toString() const = 0;

  /**
   * Converts a State into human-readable form, using names (if a name mapping is available) or else integers for each
   * state value.private
   *
   * @return
   */
  virtual std::string toString(const FeatureValue& state) const = 0;

  /**
   * Prints this object.
   */
  friend std::ostream& operator<<(std::ostream& os, const FeatureSpace& ss)
  {
    os << ss.toString();
    return os;
  }

  virtual shared_ptr<FeatureValue> readFeatureValue(const std::string& str) const = 0;

  virtual bool operator==(const FeatureSpace &ss) const = 0;
  bool operator!=(const FeatureSpace &ss) const{ return ! operator==(ss); };

  /**
   * @return Number of states, or CONTINUOUS.
   */
  virtual unsigned int getNumValues() const = 0;

  bool isInit() const{ return is_init_; }
protected:
  bool is_init_;
};

} /* namespace pomdp */
#endif /* FEATURE_SPACE_H */
