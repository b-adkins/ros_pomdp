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
 * @class pomdp::SimpleFeatureSpace
 *
 * A feature space that only has a single Feature.
 *
 * @date Aug 22, 2013
 * @author Bea Adkins
 */

#ifndef SIMPLE_FEATURE_SPACE_H
#define SIMPLE_FEATURE_SPACE_H

#include <string>
#include <vector>

#include "feature.h"
#include "feature_space.h"

// No need to tangle dependencies.
namespace XmlRpc
{
  class XmlRpcValue;
}

namespace pomdp
{
class FeatureValue;
class SimpleFeatureValue;
class BeliefFeatureValue;


class SimpleFeatureSpace : public FeatureSpace
{
public:
  SimpleFeatureSpace();
  SimpleFeatureSpace(unsigned int num_states, shared_ptr<const Pretty> name_mapping = Pretty::empty);
  SimpleFeatureSpace(unsigned int num_states, const std::vector<std::string>& name_mapping);
  virtual ~SimpleFeatureSpace();

  virtual bool loadFromYAML(const YAML::Node& feature_space_root);

  virtual bool isValid(const FeatureValue& state) const;
  virtual bool isValid(const SimpleFeatureValue& state) const;
  virtual bool isValid(const BeliefFeatureValue& belief) const;
  virtual std::string toString() const;
  virtual std::string toString(const FeatureValue& state) const;
  virtual std::string toString(const SimpleFeatureValue& state) const;
  virtual std::string toString(const BeliefFeatureValue& belief) const;

  virtual shared_ptr<FeatureValue> readFeatureValue(const std::string& str) const;

  virtual bool operator==(const FeatureSpace &ss) const
  {
    // Convert to this class type.
    try
    {
      const SimpleFeatureSpace& ss_p = dynamic_cast<const SimpleFeatureSpace&>(ss);
      // Same type, compare apples to apples.
      return feature_ == ss_p.feature_;
    }
    // Different types.
    catch(std::bad_cast& e)
    {
      return false; // apples != oranges.
    }
  }
  using FeatureSpace::operator!=;
  virtual unsigned int getNumValues() const{ return feature_.num_states_; }
  /** 
   * Serializes this object for the ROS param server.
   *
   * @todo Refactor to abstract superclass, always load them from param server.
   */
  friend XmlRpc::XmlRpcValue& operator<<(XmlRpc::XmlRpcValue& val, const SimpleFeatureSpace& s)
  {
    s.toXmlRpcValue(val);
    return val;
  }
protected:
  Feature feature_;
  virtual void toXmlRpcValue(XmlRpc::XmlRpcValue& v) const;
};

} /* namespace pomdp */
#endif /* SIMPLE_FEATURE_SPACE_H */
