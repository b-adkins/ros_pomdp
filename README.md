# POMDP for ROS                                    
## Version 0.1.0                                

## by Bea Adkins                                 
### University of Southern California                       
### Center for Robotics and Embedded Systems                   
### Interaction Lab                                
### Based on POMDP Solve by Anthony R. Cassandra                 

Package containing ROS nodes and library(ies) to facilitate the use and development of POMDPs. 

Throughout this document, ```[WORKSPACE_ROOT]``` is used for the root directory of the catkin workspace.

## Index of packages
| :--------------| :-------------------------------------------------------------------------------------|
|pomdp           | C++ classes implementing and using POMDPs.                                            |
|pomdp_node      | ROS nodes and using POMDPs.                                                           |
|                |                                                                                       |
|pomdp_reasoning | (TODO) Reasoning and learning using POMDPs.                                           |

## Index of nodes (in pomdp_node)
| :-----------| :---------------------------------------------------------------------|
|pomdp_run    | For running a single POMDP.                                           |
|oracle_irl   | Inverse reinforcement learning of policy using human oracle           |
|dialogue_irl | Inverse reinforcement learning of policy using dialogue system output |

## Compile Instructions
A copy of A. Cassandra's pomdp-solve is required. This software was developed using version 5.3. A copy of libmdp must be compiled from the source. Download and documentation at: http://www.pomdp.org/code/index.shtml .

To compile, src/pomdp/src/CMakeLists.txt must be updated with the correct location of libmdp:

-   LIB_POMDP_SOLVE_INCLUDE_DIR should contain a path to pomdp-solve/src

Compile with the following, run from [WORKSPACE_ROOT]:

    $ catkin_make

## Workspace setup
To allow ROS tools to find package directories, add the following line to your ~/.bashrc file:

    source [WORKSPACE_ROOT]/devel/setup.bash

(If rospack find, roscd, and other tools send you to ```[WORKSPACE_ROOT]/build/repo/package/catkin_generated/stamps/package```, then you should remove paths from ```$ROS_PACKAGE_PATH``` that include parent directories of ```[WORKSPACE_ROOT]```.)

## Documentation Generation
ros_pomdp uses Doxygen run through rosdoc_lite to automatically build its documentation. To do so, run the following commands from [WORKSPACE_ROOT]:

    $ rosdoc_lite src/pomdp
    $ rosdoc_lite src/pomdp_node

Documentation will be generated under ```[WORKSPACE_ROOT]/doc```. The following index files provide good starting places:

Index to node-level interface (messages and services).

-   ```[WORKSPACE_ROOT]/doc/html/index-msg.html```

Index of APIs for each package. (Links do not work in local browsing/if directories don't default to 'index.html', and currently only contains the last generated package.)

-   ```[WORKSPACE_ROOT]/doc/html/index.html```

Index of package APIs.

-   ```[WORKSPACE_ROOT]/doc/html/pomdp/index.html```
-   ```[WORKSPACE_ROOT]/doc/html/pomdp_node/index.html```

## Unit Test Instructions
Compile tests with (from ```[WORKSPACE_ROOT]```):

    $ catkin_make tests

Run tests with:

    $ catkin_make run_tests

If rostests (node level) fail because roscore is already running, force a single job. (This bug will be fixed in next ROS release.)

    $ catkin_make run_tests -j1
