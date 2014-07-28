#!/usr/bin/python

##
# @file
#
# Default settings for POMDP rostopics
#
# @author Bea Adkins
# @date 11-23-2013

# WARNING: DO NOT EDIT THESE WITHOUT ALSO UPDATING THE C++ VERSIONS.

##
# Default topic to which POMDP actions will be posted.
# 
ACTION_TOPIC = "action";

##
# Default topic to which POMDP observations will be posted.
# 
OBSERVATION_TOPIC = "observation";


## @todo How big should the queues be? POMDPs should run in real-time - otherwise the [belief] state and especially
# actions will be out of date. On the other hand, perhaps the state backlog is valuable to better determine the
# belief state?

##
# Default queue length for action publishers.
# 
ACTION_QUEUE = 1;

##
# Default queue length for observation subscribers.
# 
OBSERVATION_QUEUE = 1;