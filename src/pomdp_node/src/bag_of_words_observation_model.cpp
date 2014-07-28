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
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

// Compile out debug printing.
//#define ROS_DEBUG_STREAM(arg)

// Debug printing with ROS.
#ifdef ROSCPP_ROS_H
  #include "ros/console.h"
#else
// Debug printing with standard C++.
  #include <iostream>
  #define ROS_DEBUG_STREAM(arg) std::cout << arg << std::endl;
#endif

#include "bag_of_words_observation_model.h"

namespace pomdp
{

BagOfWordsObservationModel::BagOfWordsObservationModel()
{
  num_states_ = 0;
  P_state_given_absent_word_ = 0.05; // Default value
}

BagOfWordsObservationModel::~BagOfWordsObservationModel()
{
}

namespace bfs = boost::filesystem;
/**
 * Helper method to find bag of words files in given directory.
 *
 * @param dir Directory path
 * @param files Returned vector of paths contained within. Sorted. Undefined on failure.
 * @return True on success.
 */
bool discoverBags(bfs::path dir, std::vector<bfs::path>& files)
{
  try
  {
    if(bfs::exists(dir))
    {
      if(bfs::is_directory(dir))
      {
        std::copy(bfs::directory_iterator(dir), bfs::directory_iterator(), back_inserter(files));
        sort(files.begin(), files.end());
      }
      // else if(bfs::is_regular_file(p))
      //@TODO Also accept files containing paths to bags.
      else
      {
        ROS_DEBUG_STREAM("'" << dir << "' is not a directory.");
        return false;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("'" << dir << "' does not exist.");
      return false;
    }
  }

  catch (const bfs::filesystem_error& err)
  {
    ROS_DEBUG_STREAM(err.what());
    return false;
  }

  return true;
}


bool BagOfWordsObservationModel::loadModel(const char* dir)
{
  // Discover bag-of-words files in dir
  std::vector<bfs::path> paths;
  if(!discoverBags(bfs::path(dir), paths))
    return false;

  // Convert to std::strings.
  std::vector<std::string> paths_stl;
  for(int i = 0; i < paths.size(); i++)
    paths_stl.push_back(paths[i].string());

  return loadModel(paths_stl);
}

bool BagOfWordsObservationModel::loadModel(const std::vector<std::string>& paths)
{
  num_states_ = paths.size();

  // Intermediate variables
  std::map<std::string, double> wc_state; // Occurences of each word in a state.
  std::map<std::string, double> wc_all;   // Occurences of each word in all states.
  std::vector<double> wc_state_total(getNumStates()); // Total number of words in each state.
  double wc_grand_total = 0; // Total number of words in all states.


  // For each file, create a state
  int s = 0;
  for(std::vector<std::string>::const_iterator path = paths.begin();
      path != paths.end();
      path++)
  {
    // Open file
    bfs::ifstream word_file;
    word_file.open(path->c_str());
    if(!word_file.good()) // Check that they're valid in advance?
      return false;

    // Clear per-state variables.
    wc_state.clear();

    // Count words
    std::string word;
    while(word_file >> word)
    {
      wc_state[word]++;
      wc_all[word]++;
      wc_state_total[s]++;
    }
    wc_grand_total += wc_state_total[s];


    // Print state and total tallies
    ROS_DEBUG_STREAM("State " << s << ": " << *path);
    ROS_DEBUG_STREAM("wc_state_total: " << wc_state_total[s]);
    ROS_DEBUG_STREAM("wc_grand_total: " << wc_grand_total);

    // Print words and its word count.
//    for(std::map<std::string,double>::iterator itr = wc_state.begin();
//        itr != wc_state.end();
//        itr++)
//    {
//      ROS_DEBUG_STREAM(itr->first << "\t" << itr->second << " / " << wc_all[itr->first]);
//    }

    // For each word, calculate conditional probabilities.
    P_word_given_state_.push_back(std::map<std::string,double>()); // Initialize this state's word->probability map.
    for(std::map<std::string,double>::iterator itr = wc_state.begin();
        itr != wc_state.end();
        itr++)
    {
      word = itr->first;

      P_word_given_state_[s][word] = wc_state[word]/wc_state_total[s];
      ROS_DEBUG_STREAM("P(" << word << " | " << s << ") = " << wc_state[word] << " / " << wc_state_total[s] << " = "
                      << P_word_given_state_[s][word]);
    }

    s++;
    ROS_DEBUG_STREAM("");
  }

//  ROS_DEBUG_STREAM(word << "\t" << P_word[word] << "\t" << P_word_given_state[word][s]);
  ROS_DEBUG_STREAM("Word probabilities.");
  for(std::map<std::string,double>::iterator itr = wc_all.begin();
        itr != wc_all.end();
        itr++)
  {
    std::string word = itr->first;
    P_word_[word] = wc_all[word]/wc_grand_total;
    ROS_DEBUG_STREAM("P(w = " << word << ") = " << wc_all[word] << " / " << wc_grand_total << " = " << P_word_[word]);
  }
  ROS_DEBUG_STREAM("");

  ROS_DEBUG_STREAM("State probabilities.");
  for(int s = 0; s < getNumStates(); s++)
  {
    P_state_.push_back(wc_state_total[s]/wc_grand_total);
    ROS_DEBUG_STREAM("P(s = " << s << ") = " << wc_state_total[s] << " / " << wc_grand_total << " = " << P_state_[s]);
  }

  // Apply Bayes Rule
  //   P(s|wi) = P(wi|s)*P(s)/P(wi)
  ROS_DEBUG_STREAM("Conditional state probabilities.");
  for(int s = 0; s < getNumStates(); s++)
  {
    P_state_given_word_.push_back(std::map<std::string,double>()); // Initialize this state's word->probability map.
    for(std::map<std::string,double>::iterator itr = P_word_given_state_[s].begin();
        itr != P_word_given_state_[s].end();
        itr++)
    {
      std::string word = itr->first;
      P_state_given_word_[s][word] = P_word_given_state_[s][word] * P_state_[s]/P_word_[word];
      ROS_DEBUG_STREAM("P(" << word << "|" << s << ") = " << P_word_given_state_[s][word] << " * "
                      << P_state_[s] << " / " << P_word_[word] << " = " << P_state_given_word_[s][word]);
    }
  }
  ROS_DEBUG_STREAM("");

  is_init_ = true;
  return true;
}

/**
 * Uses loaded bags of words to translate a phrase into a state/action.
 *
 * @param phrase The phrase. Punctuation (except for apostrophes and acronyms) should be stripped. *
 * @return State/action most likely. -1 on error.
 */
int BagOfWordsObservationModel::translateCommand(const std::string& phrase) const
{
  if(!isInit())
    return -1;

  std::vector<double> P_state_given_phrase;
  if(!getProbActionForPhrase(phrase, P_state_given_phrase))
    return -1;

  std::vector<double>::iterator P_max = std::max_element(P_state_given_phrase.begin(), P_state_given_phrase.end());
  return P_max - P_state_given_phrase.begin();
}

/**
 * Estimates probability of each action for a given phrase.
 *
 * @param[in] phrase The phrase. Punctuation (except for apostrophes and acronyms) should be stripped.
 * @param[out] P_state_given_phrase P(a|ph).
 * @return True on success.
 */
bool BagOfWordsObservationModel::getProbActionForPhrase(const std::string& phrase,
                                                        std::vector<double>& P_state_given_phrase) const
{
  if(!isInit())
    return false;

  std::vector<std::string> words;
  namespace bstr = boost::algorithm;
  bstr::split(words, phrase, bstr::is_any_of(" \t"));

  return getProbActionForPhrase(words, P_state_given_phrase);
}

bool BagOfWordsObservationModel::getProbActionForPhrase(const std::vector<std::string>& phrase,
                                                        std::vector<double>& P_state_given_phrase) const
{
  P_state_given_phrase.clear();
  P_state_given_phrase.resize(getNumStates(), 1);
  for(int s = 0; s < getNumStates(); s++)
  {
    std::string word;
    namespace bstr = boost::algorithm;

    std::stringstream debug_msg;
    debug_msg << "P(" << s << " | " << bstr::join(phrase, " ") << ") = ";
    for(std::vector<std::string>::const_iterator itr = phrase.begin();
        itr != phrase.end();
        itr++)
    {
      std::string word = *itr;

      std::map<std::string, double>::const_iterator map_itr = P_state_given_word_[s].find(word);
      double P_state_given_word;
      // Get P(s|w) if it exists
      if(map_itr != P_state_given_word_[s].end())
        P_state_given_word = map_itr->second;
      // Or use low value if it doesn't
      else
        P_state_given_word = P_state_given_absent_word_;

      // Multiply in word probability
      P_state_given_phrase[s] *= P_state_given_word;
      debug_msg << P_state_given_word << " * ";
    }

    debug_msg << " = " << P_state_given_phrase[s];
    ROS_DEBUG_STREAM(debug_msg.str());
  }

  return true;
}

} /* namespace pomdp */

