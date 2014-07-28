/**
 * @file
 *
 * Helper methods for a common command line interface.
 *
 * @date Jul 20, 2013
 * @author Bea Adkins
 */

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/units/detail/utility.hpp>
#include <string>
#include <vector>

#include "pomdp.h"
#include "policy.h"

namespace pomdp
{

/** @todo Refactor these constants to a "default communication options" file. */

// WARNING: DO NOT EDIT THESE WITHOUT ALSO UPDATING THE PYTHON VERSIONS.

/**
 * Default topic to which POMDP actions will be posted.
 */
const std::string DEFAULT_ACTION_TOPIC = "action";

/**
 * Default topic to which POMDP observations will be posted.
 */
const std::string DEFAULT_OBSERVATION_TOPIC = "observation";


/** @todo How big should the queues be? POMDPs should run in real-time - otherwise the [belief] state and especially
 * actions will be out of date. On the other hand, perhaps the state backlog is valuable to better determine the
 * belief state?
 */

/**
 * Default queue length for action publishers.
 */
const unsigned int DEFAULT_ACTION_QUEUE = 1;

/**
 * Default queue length for observation subscribers.
 */
const unsigned int DEFAULT_OBSERVATION_QUEUE = 1;

bool parseArgs(int argc, char* argv[], std::vector<std::string>& mdp_paths, std::vector<std::string>& policy_paths);
bool parseArgs(int argc, char* argv[], std::vector<std::string>& mdp_paths, std::vector<std::string>& policy_paths,
               std::vector<std::string>& learning_paths);
bool loadPOMDPs(const std::vector<std::string> &paths, std::vector<MDP*>& mdps);

} /* namespace pomdp */

#endif /* COMMAND_LINE_H */
