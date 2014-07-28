/**
 * @file
 *
 * Executable to test BagOfWordsObservationModel.
 *
 * Default logger level is ROS_INFO. To see additional information, ROS_DEBUG should be used.
 *
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

#include <iostream>
#include <string>
#include <vector>

#include "bag_of_words_observation_model.h"

using namespace pomdp;

void usage(bool passive_aggression)
{
  std::cout << "USAGE: bagowords FILES" << std::endl;
  std::cout << "  or" << std::endl;
  std::cout << "USAGE: bagowords -d DIR" << std::endl;
  std::cout << "FILES  Files each containing list of words for a state/action. ";
  if(passive_aggression)
    std::cout << "'FILES' means\n         more than one!";
  std::cout << std::endl;
  std::cout << "DIR    Directory containing nothing but bags of words." << std::endl;
  std::cout << std::endl;
  std::cout << "At the prompt, enter a phrase to obtain the probability of each command. To " << std::endl;
  std::cout << "exit, type 'exit' or 'quit'." << std::endl;
}

int main(int argc, char* argv[])
{
  if(argc > 1 && (std::string(argv[1]) == "help" || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"))
  {
    usage(false);
    exit(EXIT_SUCCESS);
  }
  else if(argc == 2)
  {
    usage(true);
    exit(EXIT_FAILURE);
  }
  else if(argc < 3)
  {
    usage(false);
    exit(EXIT_FAILURE);
  }

  // Support reading from directories
  bool load_from_dir = false;
  std::string dir_path;
  std::cout << std::string(argv[1]) << " " << ((std::string(argv[1]) == "-d") ? "true" : "false") << std::endl;
  if(std::string(argv[1]) == "-d")
  {
    load_from_dir = true;
    if(argc > 2)
    {
      dir_path = argv[2];
    }
    else
    {
      std::cout << "'-d' flag read but no DIR given!" << std::endl;
      usage(false);
      exit(EXIT_FAILURE);
    }
  }

  // Load model
  BagOfWordsObservationModel model;
  std::vector<std::string> paths;
  bool success = false;
  if(load_from_dir)
  {
    success = model.loadModel(dir_path.c_str());
  }
  else
  {
    paths.assign(argv + 1, argv + argc);
    success = model.loadModel(paths);
  }
  if(!success)
  {
    std::cout << "Unable to load files:";
    for(unsigned int i = 0; i < paths.size(); i++)
    {
      std::cout << " '" << paths[i] << "'";
      if(i < paths.size() - 1)
        std::cout << ",";
    }
    std::cout << std::endl;
    exit(EXIT_FAILURE);
  }

  // Get phrases from user
  std::string phrase;
  while(true)
  {
    std::cout << ":-o < ";
    getline(std::cin, phrase);

    if(phrase == "exit" || phrase == "quit")
      exit(EXIT_SUCCESS);

    std::vector<double> P_state_given_phrase;
    if(!model.getProbActionForPhrase(phrase, P_state_given_phrase))
    {
      std::cout << "Error." << std:: endl;
    }
    else
    {
      std::cout << "P(s|\"" << phrase << "\") = [ ";
      for(std::vector<double>::iterator itr = P_state_given_phrase.begin();
          itr != P_state_given_phrase.end();
          itr++)
      {
        std::cout << *itr << " ";
      }
      std::cout << "]" << std::endl;
    }
  }

  exit(EXIT_SUCCESS);
}
