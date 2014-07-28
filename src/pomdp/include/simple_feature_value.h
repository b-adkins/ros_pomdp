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
