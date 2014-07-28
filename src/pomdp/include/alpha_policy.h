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
 * @class pomdp::AlphaPolicy
 *
 * POMDP policy using alpha tables and belief states.
 *
 * Dot-multiples the belief state (\f$ \vec b \f$) and alpha vectors (\f$ \vec \alpha_i \f$), and chooses the action
 * corresponding to the highest value. I.e.:
 * \f[ a = \Pi(\vec b) = argmax_a( \vec b \cdot \vec \alpha_i ) \f]
   \f[ i \in 0, 1, ... n \f]
 *
 * Uses extended version of A. Cassandra's Alpha file format that supports
 * comments and actions names can be used instead of numbers.
 * each alpha
 * vector is of the form:
 * @code
 * # a_x:
 * # alpha_0 alpha_1 alpha_2 ... alpha_n
 *
 * # Inline comments using hashes.
 * # E.g.:
 *
 * 0: # aka turn_right
 * .1 .2 .25 0 .45
 *
 * turn_left:
 * 0 0 .3333 .6 .06666
 *
 * @endcode
 *
 * @date Jul 15, 2013
 * @author Bea Adkins
 */

#ifndef ALPHA_POLICY_H
#define ALPHA_POLICY_H

#include "pomdp_policy.h"

namespace pomdp
{

class POMDP;

class AlphaPolicy : public POMDPPolicy
{
public:
  AlphaPolicy();
  AlphaPolicy(const char* path);
  virtual ~AlphaPolicy();
  virtual bool loadFromFile(const char* path);
  using Policy::loadFromFile;

  using POMDPPolicy::applyPolicy;
  virtual int applyPolicy(const FeatureValue& belief) const;
private:
  std::vector< std::vector<double> > alphas_;
};

} /* namespace pomdp */

#endif /* ALPHA_POLICY_H */
