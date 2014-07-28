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
 * @class pomdp::BagOfWordsObservationModel
 *
 * Calculates the probability that an input phrase corresponds to a given value of a feature (e.g. a state or an
 * action), given a bag of words for each feature value.
 *
 * One file per feature value (e.g. left, right, forward, back), containing phrases.
 *
 * E.g. left.txt:
 * @code
 * turn left
 * left
 * left turn
 * rotate left
 * turn counter-clockwise
 * port
 * .
 * .
 * .
 * @endcode
 *
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

#ifndef BAG_OF_WORDS_OBSERVATION_MODEL_H
#define BAG_OF_WORDS_OBSERVATION_MODEL_H

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>
#include <map>
#include <sstream>
#include <vector>

#include <observation_model.h>

namespace pomdp
{

class BagOfWordsObservationModel : public pomdp::ObservationModel
{
public:
  BagOfWordsObservationModel();
  virtual ~BagOfWordsObservationModel();
  bool loadModel(const char* dir);
  bool loadModel(const std::vector<std::string>& paths);

  int translateCommand(const std::string& phrase) const;

  bool getProbActionForPhrase(const std::string& phrase, std::vector<double>& P_state_given_phrase) const;
  bool getProbActionForPhrase(const std::vector<std::string>& phrase, std::vector<double>& P_state_given_phrase) const;

  int getNumStates() const{ return num_states_; };
  double getAbsentWordProbability() const{ return P_state_given_absent_word_; };
  void setAbsentWordProbability(double P_state_given_absent_word){ P_state_given_absent_word_ = P_state_given_absent_word; };
private:
  std::map<std::string,double> P_word_; /**< P(w) */
  std::vector<std::map<std::string,double> > P_word_given_state_; /**< P(w|s) */
  std::vector<double> P_state_; /**< P(s) */
  std::vector<std::map<std::string,double> > P_state_given_word_; /**< P(s|w) */

  int num_states_;
  double P_state_given_absent_word_;
};

} /* namespace pomdp */

#endif /* BAG_OF_WORDS_OBSERVATION_MODEL_H */
