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
# Generic node-level POMDP test executable. Client for a pomdp_run node.
#
# Arguments (comprising the test case) can be passed from a rostest file or the
# command line.
#
# @author Bea Adkins
# @date 2013-07-29
#

PKG = 'pomdp_node'
import roslib; roslib.load_manifest(PKG)

import exceptions
import sys
from os.path import basename
import unittest

import rospy
import rostest

from pomdp_node import default
from pomdp_node import msg

NODE_NAME = 'test_pomdp_run'

def usage():
    return ("USAGE: %s Z0 A0 Z1 A1...\n"
            "For all i, tests Zi -> POMDP -> Ai.\n"
            "  Zi - observations sent (integers)\n"
            "  Ai - action expected (integers), or -1 for failure"%basename(sys.argv[0]))
#@todo Add service name.

## Utility function that tests if an object can be converted to an integer.
#
# @param obj Object to be tested as an integer.
# @return True if obj can be converted.
def isInt(obj):
    try:
        int(obj)
        return True
    except ValueError:
        return False


class TestPOMDPRun(unittest.TestCase):
    def __init__(self, *args):
        super(TestPOMDPRun, self).__init__(*args)
        rospy.init_node(NODE_NAME)
        self.args = args
        
        # Initialize communication
        self.observation_pub = rospy.Publisher(default.OBSERVATION_TOPIC, msg.observation)        
        self.action_sub = rospy.Subscriber(default.ACTION_TOPIC, msg.action, self.save_action)
        rospy.sleep(.5) # Give pub and sub time to start.
                
    ## Saves action from Publisher
    def save_action(self, msg):
        self.test_action = msg.action
                
    def test_pomdp_run(self):
        # Get command line arguments
        args = rospy.myargv()

        # Read integers
        numbers = [int(arg) for arg in args if isInt(arg)]
        if(len(numbers)%2 != 0):
            raise ValueError("Test arguments must contain equal numbers observations and actions.")
        if(len(numbers) < 2):
            raise ValueError("Test arguments need at least one observation -> action pair.")
        
        # Pair alternating integers.
        obs_to_actions = zip(numbers[0::2], numbers[1::2])

        # Call the actual test function for each pair
        for index,obs_to_action in enumerate(obs_to_actions, start=1):
            self._test_pomdp_run(index, *obs_to_action)        
        
##
# Runs one cycle of a POMDP using the observation and action publish/subscribers and checks its 
# returned action.
#
# @param index Index of this observation -> action pair.
# @param observation Input observation. (Integer.) 
# @param expected_action Expected action. (Integer.) Or -1 for an expected error. 
#
    def _test_pomdp_run(self, index, observation, expected_action):
        # Publish recorded observation
        self.observation_pub.publish(msg.observation(observation))
        
        # Give the subscriber time to return
        TIMEOUT = .5
        try:
            rospy.wait_for_message(default.ACTION_TOPIC, msg.action, timeout=TIMEOUT)
        # Timeout
        except rospy.ROSException:
            # Timeout expected
            if expected_action == -1:
                return # Pass
            # Timeouts are an error
            else:
                self.fail("Unexpected timeout on action topic %s" % (action_sub.getTopic()))

        # Check returned action.
        self.assertEqual(self.test_action, expected_action, 
                         "For #%i: observation %i, returned action '%i' != expected action '%i'!" 
                           % (index, observation, self.test_action, expected_action))

if __name__ == '__main__':
    # Display help message.
    if(sys.argv[1] in ["help", "--help", "-h"]):
        #rospy.loginfo("\n%s", usage());
        print usage()
        sys.exit(0)

    rostest.rosrun(PKG, NODE_NAME, TestPOMDPRun, sys.argv) 
