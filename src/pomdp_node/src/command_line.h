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
