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
# Generic node-level Policy inverse reinforcement learning test executable. 
# Client for an dialogue_irl node.
#
# Arguments (a test case file) can be passed from a rostest file or the command
# line.
#
# @author Bea Adkins
# @date 2013-08-06
#

PKG = 'pomdp_node'
import roslib; roslib.load_manifest(PKG)

import exceptions
import numpy as np
import numpy.testing as npt
import re
import sys
from os.path import basename
import unittest

import rospy
import rostest

from pomdp_node.srv import dialogue_irl

NODE_NAME = 'test_dialogue_irl'
SERVICE_NAME = 'dialogue_irl'

def usage():
    return ("USAGE: %s FILE\n"
"  FILE - test case file. One column for each element of service arguments and\n"
"    expected return. So:\n"
"[obs] [phrase] [P_model_0] [P_model_1] ... [action_0] [action_1] ...\n"
"  phrase - words should be joined by commas without spaces.\n"
"Comments marked by '#' before any non-whitespace characters."%basename(sys.argv[0]));

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

## Utility function that tests if an object can be converted to a float.
#
# @param obj Object to be tested as a float.
# @return True if obj is a float.
def isFloat(obj):
    try:
        float(obj)
        return True
    except ValueError:
        return False

class TestDialogueIRL(unittest.TestCase):
    def __init__(self, *args):
        super(TestDialogueIRL, self).__init__(*args)
        rospy.init_node(NODE_NAME)
        self.args = args

    def test_dialogue_irl(self):
        # Get command line arguments
        args = rospy.myargv()
        path = args[1]
        
        # Read file
        file = open(path, 'r')
        
        # Peek at first data line to discover number of columns
        for line in file:            
            # Comment line, skips.
            if(re.search("^\s*#", line)):
                continue
        
            # Data line trips.
            if(re.search("(\s*[0-9\.]+\s+)+", line)):
               # In-line comment, strips.
               line = re.sub("#.*$", "", line)
               
               # Count data elements
               n_cols = len(line.split())
               
               break
        file.seek(0) # Return to start of file
        
        # Read data.
        obs = np.genfromtxt(path, usecols=(0), dtype='int', comments='#')
        phrases = np.genfromtxt(path, usecols=(1), dtype='string', comments='#')
        vals = np.genfromtxt(path, usecols=(range(2, n_cols)), dtype='float', comments='#')
                        
        # Check data dimensions
        n_rows = len(vals)
        self.assertGreaterEqual(n_cols, 6, "Need at least six columns of data for meaningful test file.")
        self.assertGreaterEqual(n_rows, 1, "Need at least one row of data for meaningful test file.")
        self.assertEquals(n_cols % 2, 0, "Mismatch between size of model likelihood and size of actions.")
        n_models = (n_cols - 2)/2
        self.assertGreaterEqual(n_models, 2, "Need at least two models for meaningful test file.")
        
        # Slice data and convert type.
        p_models = vals[:, :n_models].astype(float)
        a_models = vals[:, n_models:].astype(int)
        phrases = [phrase.split(',') for phrase in phrases]

#         self.fail("data: %s\n%s\n%s\n%s\n"%(obs, phrases, p_models, a_models))
        
        # Call the actual test function for each row
        for i in range(0, n_rows):
            self._test_oracle_irl(i, obs[i], phrases[i], p_models[i], a_models[i])

##
# Runs on cycle of IRL of a policy with an oracle and checks returned moedel
# likelihoods and actions.
#
# @param index Row index.
# @param obs Input obersvation. (Integer.)
# @param phrase Oracle phrase. (Integer.)
# @param p_models_exp Expected model likelihoods.
# @param a_models_exp Expected models' actions.
#
    def _test_oracle_irl(self, index, obs, phrase, p_models_exp, a_models_exp):
        rospy.wait_for_service(SERVICE_NAME, 5)
        srv = rospy.ServiceProxy(SERVICE_NAME, dialogue_irl)
        rospy.wait_for_service(SERVICE_NAME, 5)
        rsp = srv(obs, phrase)
        p_models = np.float_(rsp.model_likelihoods)
        a_models = np.int_(rsp.model_actions)
        
        # Compare expected to actual values.
        try:
            # Use Numpy assert to trigger a Python assert (thus message fields are empty).
            # Hacky version uses noiseless oracle data with very loose tolerances.
            npt.assert_allclose(p_models, p_models_exp, 1e-1, 1e-1, "", 0)
        except AssertionError, e:
            self.fail("@%i (z = %i, ph = '%s'): Returned P(model) != expected P(model):\n%s != %s"
                         %(index, obs, " ".join(phrase), p_models, p_models_exp))
        try:
            # Use Numpy assert to trigger a Python assert (thus message fields are empty).
            npt.assert_array_equal(a_models, a_models_exp, "", 0)
        except AssertionError, e:
            self.fail("@%i (z = %i, ph = '%s'): Returned action != expected action:\n%s != %s"
                         %(index, obs, " ".join(phrase), a_models, a_models_exp))

if __name__ == '__main__':
    # Display help message.
    if(len(sys.argv) < 2 or sys.argv[1] in ["help", "--help", "-h"]):
        #rospy.loginfo("\n%s", usage());
        print usage()
        sys.exit(0)
    
    rostest.rosrun(PKG, NODE_NAME, TestDialogueIRL, sys.argv)
