/**
 * @class pomdp::VectorFeatureSpace
 *
 * Represents a Feature composed of multiple sub-states.
 *
 * @todo Make this work with POMDPs too.
 *
 * @date Aug 22, 2013
 * @author Bea Adkins
 */

#ifndef VECTOR_FEATURE_SPACE_H
#define VECTOR_FEATURE_SPACE_H

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "feature_space.h"
#include "feature_vector.h"

using boost::shared_ptr;
using boost::weak_ptr;

namespace pomdp
{
class FeatureValue;
class SimpleFeatureValue;
class BeliefFeatureValue;

class VectorFeatureSpace : public FeatureSpace
{
public:
  /** Empty. */
  VectorFeatureSpace() : feature_vector_(FeatureVector::empty){};
  /** Create from existing FeatureVector. */
  VectorFeatureSpace(const FeatureVector& fv)
    : feature_vector_(fv)
  {
    is_init_ = (feature_vector_.size() > 0) ? true : false;
  }
  /** Create from FeatureVector file. Will be empty (getNumStates() == 0) on failure. */
  VectorFeatureSpace(const char* path);
  /** Create from FeatureVector file. Will be empty (getNumStates() == 0) on failure. */
  VectorFeatureSpace(const std::string& path){ VectorFeatureSpace(path.c_str()); }

  virtual bool loadFromYAML(const YAML::Node& feature_space_root);

  virtual bool isValid(const FeatureValue& state) const;
  virtual bool isValid(const SimpleFeatureValue& state) const;
  virtual bool isValid(const BeliefFeatureValue& belief) const;
  virtual bool isValid(const FeatureVectorValue& state) const;

  virtual FeatureValue hash(const FeatureValue& state) const;
  virtual SimpleFeatureValue hash(const SimpleFeatureValue& state) const;
  virtual BeliefFeatureValue hash(const BeliefFeatureValue& belief) const;
  virtual SimpleFeatureValue hash(const FeatureVectorValue& state) const;

  virtual FeatureValue dehash(const FeatureValue& state) const;
  virtual FeatureVectorValue dehash(const SimpleFeatureValue& state) const;
  virtual BeliefFeatureValue dehash(const BeliefFeatureValue& belief) const;
  virtual FeatureVectorValue dehash(const FeatureVectorValue& state) const;

  virtual std::string toString() const;
  virtual std::string toString(const FeatureValue& state) const;
  virtual std::string toString(const SimpleFeatureValue& state) const;
  virtual std::string toString(const BeliefFeatureValue& belief) const;
  virtual std::string toString(const FeatureVectorValue& state) const;

  virtual shared_ptr<FeatureValue> readFeatureValue(const std::string& str) const;

  virtual bool operator==(const FeatureSpace &ss) const
  {
    // Convert to this class type.
    try
    {
      const VectorFeatureSpace& ss_p = dynamic_cast<const VectorFeatureSpace&>(ss);
      // Same type, compare apples to apples.
      return feature_vector_ == ss_p.feature_vector_;
    }
    // Different types.
    catch(std::bad_cast& e)
    {
      return false; // apples != oranges.
    }
  }
  using FeatureSpace::operator!=;
  virtual unsigned int getNumValues() const{ return feature_vector_.getNumStates(); }
protected:
  FeatureVector feature_vector_;
};

} /* namespace pomdp */
#endif /* VECTOR_FEATURE_SPACE_H */
