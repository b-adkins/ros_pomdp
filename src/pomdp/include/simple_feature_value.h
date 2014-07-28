/**
 * @class pomdp::State
 *
 * Thin wrapper class to centralize [belief] state type checking and make applyPolicy() polymorphic.
 *
 * @date Aug 12, 2013
 * @author Bea Adkins
 */

#ifndef STATE_H
#define STATE_H

#include <ros/console.h>
#include <ros/assert.h>

#include "feature_value.h"

namespace pomdp
{

// @todo TODO Possible optimization: use smart pointers and an object pool to reduce/eliminate memory allocation and
// assignment/copy overhead and unpredictablility. This gives MDPs their fast processing advantage over POMDPs.

/**
 * Represents a Feature Value that can be one of a finite set of discrete values. E.g. an MDP state, a (PO)MDP action,
 * a POMDP observation.
 */
class SimpleFeatureValue : public FeatureValue
{
public:
  /** Constructs a DiscreteFeatureValue. */
  SimpleFeatureValue(unsigned int state);
  /** Copy constructor. */
  SimpleFeatureValue(const SimpleFeatureValue& s);
  virtual ~SimpleFeatureValue(){};

  SimpleFeatureValue& operator=(SimpleFeatureValue const& s);
  SimpleFeatureValue& operator=(unsigned int const& s);

  /**
   * Use this struct as an int.
   *
   * Does not check validity of data returned! That's what isValid(int) is for!
   *
   * @return State.
   */
  const unsigned int asInt() const{ return state_; };
protected:
    unsigned int state_;
};

} /* namespace pomdp */
#endif /* STATE_H */
