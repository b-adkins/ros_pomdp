/**
 * @file
 *
 * Class to convert several component MDPStates, each with a small number of possible states, to an equivalent
 * CombinedState that holds the same information in a single number.
 *
 * @deprecated Used as backend for VectorFeatureSpace, will eventually be merged with that.
 *
 * @date Aug 20, 2013
 * @author Bea Adkins
 */

#ifndef FEATURE_VECTOR_H
#define FEATURE_VECTOR_H

#include <boost/foreach.hpp>
#include <iostream>
#include <ros/assert.h>
#include <ros/console.h>
#include <string>
#include <vector>
// Written for 0.2.7-5 API
#include <yaml-cpp/yaml.h>

#include "feature.h"
#include "feature_value.h"

namespace pomdp
{

class FeatureVectorValue : public FeatureValue
{
public:
  FeatureVectorValue(const std::vector<unsigned int>& states) : values_(states){};
  FeatureVectorValue(const FeatureVectorValue& v) : values_(v.values_){};

  unsigned int size() const { return values_.size(); }
  unsigned int& operator[](unsigned int i) { ROS_ASSERT(i < size()); return values_[i]; }
  unsigned int const& operator[](unsigned int i) const { ROS_ASSERT(i < size()); return values_[i]; }

//  FeatureVectorValue& operator=(const FeatureVectorValue& val)
//  {
//    if(&val != this)
//    {
//      values_ = val.values_;
//    }
//    return *this;
//  }

//  FeatureVectorValue& operator=(const std::vector<unsigned int>& values)
//  {
//    values_ = values;
//    return *this;
//  }

  bool operator==(const FeatureVectorValue& v) const
  {
    return values_ == v.values_;
  }
  bool operator!=(const FeatureVectorValue& v) const{ return ! operator==(v); }
protected:
  std::vector<unsigned int> values_;
};


/**
 * Maps a vector of sub-states, one for each vector, to a single state value.
 *
 * @section mapping Mapping
 *
 * The mapping uses an extension of positional numeral systems (e.g. binary, decimal) where each "digit" (corresponding
 * to a single Feature) can have a different base.
 *
 * Place value is calculated by:
 * \f[ \nu[i] =
     \left\{
       \begin{array}{lr}
         1                       & : i = 0 \\
         \prod_{j=0}^{f-1} n_{j} & : i \ge 1
       \end{array}
     \right.
   \f]
 *
 * And numeric value by:
 *
 * \f[ S = \vec{s} \cdot \vec{\nu} \f]
 * \f[ S = s_0 + s_1 n_0 + s_2 n_0 n_1 + ... + s_f \prod_{j=0}^{f-1} n_{j} \f]
 *
 * Where \f$ n_f \f$ is number of states in Feature #f, \f$ \vec s \f$ is current compound state.
 *
 * Example:
 *
 * FeatureVector
 *
 * \f$ \vec n   = [ 4 , 2 , 3   ] \f$
 *
 * \f$ \vec \nu = [ 1 , 4 , 4 \cdot 2 ] = [ 1 , 4 , 8 ] \f$
 *
 * FeatureVectorValue
 *
 * \f$ \vec s   = [ 3 , 1 ,  2  ] \f$
 *
 * numeric state
 *
 * \f$ \vec s \cdot \vec \nu = 3\cdot1 + 1\cdot4 + 8\cdot2 = 3 + 4 + 16 = 23 \f$
 *
 *
 * The convention forNum this class is that, unlike arabic numberals, the right-most Features (last added to the
 * FeatureVector) are given the highest place value.
 *
 * @section file_fmt File Format
 *
 * Simple YAML list whose elements have two attributes - name and state enumeration. The 'states' fields accept
 * numeric or nominal values - the number of states or the names of each state, respectively. Again, the last features
 * added (last in the list definition) are given the highest place value.
 *
 *  E.g. 4x4_ss.yaml
 * @code
#         x
#     0  1  2  3
#    +-----------+
#  0 | 0  1  2  3|
#y 1 | 4  5  6  7|
#  2 | 8  9 10 11|
#  3 |12 13 14 15|
#    +-----------+
#
features:
  - name: x
    states: 4
  - name: y
    states: 4
 * @endcode
 *
 * @todo Generalize to hierarchical states? I.e. FeatureTree.
 */
class FeatureVector
{
public:
  FeatureVector(){};
  FeatureVector(unsigned int n, struct Feature* features) : features_(features, features + n)
  {
    generatePlaceValue();
  }
  FeatureVector(const std::vector<Feature>& features) : features_(features)
  {
    generatePlaceValue();
  }
  inline virtual ~FeatureVector(){};

  friend void operator>>(const YAML::Node& node, FeatureVector& fv);

  /**
   * Adds feature to the end of the feature vector. Doesn't allows duplicates.
   *
   * @param f New feature to add.
   * @return True on success.
   */
  bool addFeature(const Feature& f)
  {
    // Fail if feature already exists.
    if(std::find(features_.begin(), features_.end(), f) != features_.end())
      return false;

    features_.push_back(f);
    generatePlaceValue(); // Effective but inefficient to repeat from scratch.
    return true;
  }

  /**
   * Removes feature from the feature vector.
   *
   * @param f FeatureNum to remove.
   * @return True on success.
   */
  bool removeFeature(const Feature& f)
  {
    std::vector<Feature>::iterator erase_me = std::find(features_.begin(), features_.end(), f);

    // Feature not found.
    if(erase_me == features_.end())
      return false;
    // Feature found, erase it.
    else
      features_.erase(erase_me);

    generatePlaceValue(); // Effective but inefficient, possibly unavoidable, to repeat from scratch.
    return true;
  }

  /**
   * Whether a FeatureVectorValue is valid for this FeatureVector.
   *
   * Valid if both have the same size and all states are within range.
   *
   * @param val Value under test.
   * @return True for valid.
   */
  bool validValue(const FeatureVectorValue& val) const
  {
    // Have to have same number of features.
    if(val.size() != size())
      return false;

    // State of each feature has to be within valid range.
    for(int f = 0; f < size(); f++)
      if(val[f] >= features_[f].num_states_)
        return false;

    return true;
  }

  /**
   * Converts FeatureVectorValue into numeric state.
   *
   * @states Valid FeatureVectorValue.
   * @return Numeric state. Zero if FeatureVector is empty.
   */
  unsigned int toStateNumber(const FeatureVectorValue& states) const
  {
    ROS_ASSERT(validValue(states));

    if(size() == 0)
      return 0;

    // Dot product.
    unsigned int sum = 0;
    for(int f = 0; f < size(); f++)
    {
      sum += states[f] * place_values_[f];
    }

    return sum;
  }

  /**
   *
   *
   * @simple_state Numeric state.
   * @compound_state FeatureVectorValue.
   */
  FeatureVectorValue toFeatureVectorValue(unsigned int simple_state) const
  {
    ROS_ASSERT(simple_state <= getMaxStateNumber());

    std::vector<unsigned int> compound_state(size(), 0);

    unsigned int remainder = simple_state; //4noobs will compile out, reduces confusion.

    // Countdown loop.
    for(int f = size() - 1; f >= 0; f--)
    {
       compound_state[f] = remainder / place_values_[f]; // Int division
       remainder = remainder % place_values_[f];
    }

    return FeatureVectorValue(compound_state);
  }

  /**
   * Product of all features' numbers of sub-states.
   *
   * @return Maximum value of numeric state that can be generated from this FeatureVector.
   */
  // Probably unneeded optimization: store this value upon construction.
  unsigned int getMaxStateNumber() const{ return getNumStates() - 1; /* Convert to C index */ }
  unsigned int getNumStates() const
  {
    int product = 1;
    for(int f = 0; f < size(); f++)
      product *= features_[f].num_states_;
    return product;
  }

  /**
   * @return Number of Features.
   */
  unsigned size() const{ return features_.size(); }

  /**
   * Empty FeatureVector. (size() == 0)
   */
  static const FeatureVector& empty;

  /**
   * Equal if they have the same Features in the same order (implying same FeatureVector to state number mapping).
   */
  bool operator==(const FeatureVector &ss) const{ return features_ == ss.features_; /* Use existing vector ==.*/ };
  bool operator!=(const FeatureVector &ss) const{ return ! operator==(ss); }

  std::string toString() const
  {
    std::stringstream ret;
    operator<<(ret, *this);
    return std::string(ret.str());
  }
  friend std::ostream& operator<<(std::ostream& os, const FeatureVector& fv)
  {
    os << "FeatureVector, length "  << fv.size() << ": [" << ((fv.size() > 1) ? "\n" : "");
//    BOOST_FOREACH(const Feature& f, fv.features_)
    for(int i = 0; i < fv.features_.size(); i++)
    {
      const Feature& f = fv.features_[i];
      os << f << ((fv.size() > 1) ? "\n" : "");
    }
    os << "]\n";

    return os;
  }

  virtual Feature operator[](unsigned int i) const{ ROS_ASSERT(i < size()); return features_[i]; }
protected:
  std::vector<Feature> features_; /**< Each component feature. */
  std::vector<int> place_values_; /**< Speeds up conversion between FeatureVectorValue to a single state number. */

  /**
   * Initializes internal place value vector.
   *
   * @todo Make this incremental (for push_back()ed Features) instead of always starting over.
   */
  void generatePlaceValue()
  {
    place_values_.resize(features_.size());
    std::fill(place_values_.begin(), place_values_.end(), 1); // Reset place values vector.

    // Feature #0 always has place value of 1.

    // Remaining features: place_value[i] = n[i-1] * n[i-2] * ... * n[1] * n[0]
    //   Where n is number of states.
    for(int f = 1; f < size(); f++)
    {
      for(int i = 0; i <= f-1; i++)
        place_values_[f] *= features_[i].num_states_;
    }
  }
};

class FeatureVectorFactory
{
public:
  FeatureVector loadFromFile(const char* path);
  FeatureVector loadFromFile(const std::string& path){ return loadFromFile(path.c_str()); }
  FeatureVector loadFromYAML(const YAML::Node& feature_space_root);
};

} /* namespace pomdp */
#endif /* FEATURE_VECTOR_H_ */
