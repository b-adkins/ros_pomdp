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
 * @class pomdp::POMDP
 *
 * Partially Observable Markov Decision Process class.
 *
 * Enables multiple POMDPs (in A. Cassandra's .POMDP format) to be active at
 * once.
 *
 * For objects allocated on the heap (using the "new" keyword), POMDP(path)
 * should be used.
 *
 * For objects allocated on the stack, POMDP() should be used with
 * loadFromFile().
 *
 * @date Jul 9, 2013
 * @author Bea Adkins
 */

#ifndef POMDP_H
#define POMDP_H

#include "mdp.h"
#include "pomdp_policy.h"

namespace pomdp
{
// No need to tangle dependencies.
class Pretty;

class POMDP : public MDP{
public:
  POMDP();
  virtual ~POMDP();
  virtual bool loadFromFile(const char* path);
  using MDP::loadFromFile; // Use MDP's loadFromFile(std::string) to call POMDP's loadFromFile(char*)

  virtual bool updateBeliefState(int observation);
  virtual bool updateBeliefState(int observation, int action);

  virtual bool setMyPolicy(shared_ptr<const Policy> p);
  virtual int applyPolicy();

  int getNumObservations() const { return num_observations_; }
  void setPrettyObservation(shared_ptr<const Pretty> pretty){ if(!pretty) return; pretty_observation_ = pretty; };
  virtual std::string prettyObs(int observation);
protected:
  // Fields for POMDPs only.
  shared_ptr<const POMDPPolicy> my_policy_;
  int num_observations_;
  Matrix *R_;  /**< Observation Probabilities */
  shared_ptr<const Pretty> pretty_observation_; /**< Pretty printer for observations. */

  // Note: all POMDPs have a reward function!
};

} /* namespace pomdp */

#endif /* POMDP_H */
