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
# Client for an oracle_irl node.
#
# Arguments (a test case file) can be passed from a rostest file or the command
# line.
#
# @author Bea Adkins
# @date 2013-08-01
#

PKG = 'pomdp_node'
import roslib; roslib.load_manifest(PKG)

import exceptions
import numpy as np
import numpy.testing as npt
import sys
from os.path import basename
import unittest

import rospy
import rostest

from pomdp_node.srv import oracle_irl

NODE_NAME = 'test_oracle_irl'
SERVICE_NAME = 'oracle_irl'

def usage():
    return ("USAGE: %s FILE\n"
"  FILE - test case file. One column for each element of service arguments and\n"
"    expected return. So:\n"
"[obs] [a_oracle] [P_model_0] [P_model_1] ... [action_0] [action_1] ...\n"
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

class TestOracleIRL(unittest.TestCase):
    def __init__(self, *args):
        super(TestOracleIRL, self).__init__(*args)
        rospy.init_node(NODE_NAME)
        self.args = args

    def test_oracle_irl(self):
        # Get command line arguments
        args = rospy.myargv()
        path = args[1]
        
        # Read file
        try:
            vals = np.loadtxt(path, comments='#')
        except Exception, e:
            self.fail("Failed to read file: '%s'\n with exception %s"%(path, e))
            
        # Check data dimensions
        n_cols = len(vals[0])
        n_rows = len(vals)
        self.assertEquals(n_cols % 2, 0, "Mismatch between size of model likelihood and size of actions.")
        self.assertGreaterEqual(n_cols, 6, "Need at least six columns of data for meaningful test file.")
        self.assertGreaterEqual(n_rows, 1, "Need at least one row of data for meaningful test file.")
        n_models = (n_cols - 2)/2
        self.assertGreaterEqual(n_models, 2, "Need at least two models for meaningful test file.")
        
        # Slice data
        try:
            obs      = vals[:, 0]
            a_oracle = vals[:, 1]
            p_models = vals[:, 2:2+n_models]
            a_models = vals[:, 2+n_models:]
        except Exception, e:
            self.fail("Error slicing data: %s"%e)

        # Type convert data
        try:
            obs = np.int_(obs)
            a_oracle = np.int_(a_oracle)
            p_models = np.float_(p_models)
            a_models = np.int_(a_models)
        except Exception, e:
            self.fail("Error converting data: %s\n%s\n%s\n%s\n%s\n"%(e, obs, a_oracle, p_models, a_models))

        # Call the actual test function for each row
        for i in range(0, n_rows):
            self._test_oracle_irl(i, obs[i], a_oracle[i], p_models[i], a_models[i])

##
# Runs on cycle of IRL of a policy with an oracle and checks returned moedel
# likelihoods and actions.
#
# @param index Row index.
# @param obs Input obersvation. (Integer.)
# @param a_oracle Oracle action. (Integer.)
# @param p_models_exp Expected model likelihoods.
# @param a_models_exp Expected models' actions.
#
    def _test_oracle_irl(self, index, obs, a_oracle, p_models_exp, a_models_exp):
        srv = rospy.ServiceProxy(SERVICE_NAME, oracle_irl)
        rospy.wait_for_service(SERVICE_NAME, 5)
        
        rospy.logdebug("Index: %i", index)
        
        # Call service.
        rsp = srv(obs, a_oracle)
        p_models = np.float_(rsp.model_likelihoods)
        a_models = np.int_(rsp.model_actions)
        
        # Compare expected to actual values.
        try:
            # Use Numpy assert to trigger a Python assert (thus message fields are empty).
            npt.assert_allclose(p_models, p_models_exp, 1e-3, 1e-3, "", 0)
        except AssertionError, e:
            self.fail("@%i (z = %i, a_o = %i): Returned P(model) != expected P(model):\n%s != %s"
                         %(index, obs, a_oracle, p_models, p_models_exp))
        try:
            # Use Numpy assert to trigger a Python assert (thus message fields are empty).
            npt.assert_array_equal(a_models, a_models_exp, "", 0)
        except AssertionError, e:
            self.fail("@%i (z = %i, a_o = %i): Returned action != expected action:\n%s != %s"
                         %(index, obs, a_oracle, a_models, a_models_exp))

if __name__ == '__main__':
    # Display help message.
    if(len(sys.argv) < 2 and sys.argv[1] in ["help", "--help", "-h"]):
        #rospy.loginfo("\n%s", usage());
        print usage()
        sys.exit(0)

    rostest.rosrun(PKG, NODE_NAME, TestOracleIRL, sys.argv) 
