#!/usr/bin/env python

##
# @file
#
# Application that translates rostopics containing actions, states, or observations into from intergers to human-readable forms.
#
# @author Bea Adkins
# @date 03-14-2014

import argparse
import os
import sys

import rospy
from std_msgs import msg as std_msgs

# POMDP
from pomdp.feature_space import *

# POMDP Topics
from pomdp_node import default
from pomdp_node import msg as pomdp_msg

def main():
    # Parse command line arguments
    ros_argv = rospy.myargv()
    script_name = os.path.basename(ros_argv[0])
    parser = argparse.ArgumentParser(prog=script_name, description='Translates POMDP rostopics containing actions, states, or observations into human-readable form, using the supplied FeatureSpace.')
    parser.add_argument('-o', '--obs', dest='observation_topic', help='POMDP observation/MDP state topic.')
    parser.add_argument('-s', '--state', dest='state_topic', help='POMDP/MDP state topic.')    
    parser.add_argument('-a', '--action', dest='action_topic', help='POMDP action topic.')
    parser.add_argument('-O', '--obsspace', dest='observation_space_param', help='POMDP observation/MDP state space.')
    parser.add_argument('-S', '--statespace', dest='state_space_param', help='POMDP/MDP state space.')
    parser.add_argument('-A', '--actionspace', dest='action_space_param', help='POMDP action space.')
    parser.add_argument('-p', '--publish', action='store_true', dest='will_publish',
                          help='Will publish to text topics, with paths of the original topic with "_str" appended.')
    parser.add_argument('-q', '--quiet', action='store_false', dest='will_print',
                          help='Will not print subscribed topics to the screen.')
    parser.set_defaults(will_publish=False, will_print=True)
    args = parser.parse_args(ros_argv[1:])

    # Validate command line arguments
    if(not args.will_publish and not args.will_print):
        sys.stdout.write("Need some form of output (publishing or printing).")
        sys.exit(1)

    if(not(args.observation_topic or args.state_topic or args.action_topic)):
        sys.stdout.write("Need at least one observation, state, or action topic and feature space pair to print!\n")
        sys.exit(1)

    if(bool(args.observation_topic) != bool(args.observation_space_param)):
        sys.stdout.write("Need an observation space and at least one observation topic.\n")
        sys.exit(1)
    if(bool(args.state_topic) != bool(args.state_space_param)):    
       sys.stdout.write("Need a state space and at least one state topic.\n")
       sys.exit(1)
    if(bool(args.action_topic) != bool(args.action_space_param)):
       sys.stdout.write("Need an action space and at least one action topic.\n")
       sys.exit(1)


    # Initialize ROS node
    node_name = 'pomdp_viewer' # Default
    rospy.init_node(node_name)

    # Load FeatureSpaces
    fs_fact = FeatureSpaceFactory()
    if(args.observation_space_param):
        observation_space = fs_fact.getFeatureSpace(rospy.get_param(args.observation_space_param))
    if(args.state_space_param):
        state_space = fs_fact.getFeatureSpace(rospy.get_param(args.state_space_param))
    if(args.action_space_param):
        action_space = fs_fact.getFeatureSpace(rospy.get_param(args.action_space_param))

    # If publishing, create topics
    if(args.observation_topic):
        observation_str_topic = rospy.Publisher(args.observation_topic + "_str", std_msgs.String)
    if(args.state_topic):
        state_str_topic = rospy.Publisher(args.state_topic + "_str", std_msgs.String)
    if(args.action_topic):
        action_str_topic = rospy.Publisher(args.action_topic + "_str", std_msgs.String)

    # Handle observation topic
    def handle_observation_topic(msg):
        if(observation_space.is_valid(msg.observation)):
            observation_str = observation_space.toString(msg.observation)
            if(args.will_print):
                rospy.loginfo("Observation: %s" % (observation_str))
            if(args.will_publish):
                observation_str_topic.publish(observation_str)
        else:
            rospy.logwarn("Invalid observation: %i" % msg.observation)
            return

    # Handle state topic
    def handle_state_topic(msg):
        if(state_space.is_valid(msg.state)):
            state_str = state_space.toString(msg.state)
            if(args.will_print):
                rospy.loginfo("State: %s" % (state_str))
            if(args.will_publish):
                state_str_topic.publish(state_str)
        else:
            rospy.logwarn("Invalid state: %i" % msg.state)
            return    

    # Handle action topic
    def handle_action_topic(msg):
        if(action_space.is_valid(msg.action)):
            action_str = action_space.toString(msg.action)
            if(args.will_print):
                rospy.loginfo("Action: %s" % (action_str))
            if(args.will_publish):
                action_str_topic.publish(action_str)
        else:
            rospy.logwarn("Invalid action: %i" % msg.action)
            return


    # Subscribe to topics
    if(args.observation_space_param):
        observation_sub    = rospy.Subscriber(args.observation_topic, pomdp_msg.observation, handle_observation_topic)
    if(args.state_space_param):
        state_sub  = rospy.Subscriber(args.state_topic, pomdp_msg.state, handle_state_topic)
    if(args.action_space_param):
        action_sub = rospy.Subscriber(args.action_topic, pomdp_msg.action, handle_action_topic)   

    rospy.spin()

if __name__ == "__main__":
    main()
