#!/usr/bin/env python

# The MIT License (MIT)
# 
# Copyright (c) 2014 Boone "Bea" Adkins
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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