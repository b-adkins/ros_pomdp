cmake_minimum_required(VERSION 2.8.3)
project(pomdp_node)

set(ROS_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg rostest)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)

## Sister pomdp package
find_package(pomdp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   action.msg
   observation.msg
   model_output.msg
   model_outputs.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
#    lookup_trans_prob.srv
#    estimate_action.srv
#    mdp_policy.srv
#    alpha_policy.srv
#    update_belief.srv
    oracle_irl.srv
    dialogue_irl.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pomdp_node
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(src include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pomdp_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pomdp_node
#   src/${PROJECT_NAME}/pomdp_node.cpp
# )

## Declare cpp executables
add_executable(dialogue_irl src/dialogue_irl_main.cpp src/command_line.cpp
                            src/bag_of_words_observation_model.cpp src/observation_model.cpp) 
add_executable(oracle_irl src/oracle_irl_main.cpp src/command_line.cpp)
add_executable(pomdp_run src/pomdp_run_main.cpp src/command_line.cpp)
add_executable(bagowords src/bag_of_words_main.cpp src/bag_of_words_observation_model.cpp src/observation_model.cpp)

## Build properties
# For targets that use c++0x features.
set_target_properties(dialogue_irl oracle_irl pomdp_run PROPERTIES COMPILE_FLAGS -std=c++0x)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pomdp_node_node pomdp_node_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dialogue_irl
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   pomdp
   ${GSL_LIBRARIES}
   yaml-cpp
 )

target_link_libraries(oracle_irl
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   pomdp
   ${GSL_LIBRARIES}
   yaml-cpp
 )

target_link_libraries(pomdp_run
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   pomdp
   ${GSL_LIBRARIES}
   yaml-cpp
 )
 
target_link_libraries(bagowords
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pomdp_node pomdp_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pomdp_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Manually add rostests (Node-level, Python unit tests).
#
# Until catkin_make is patched, may need to run catkin_make with "-j1" to avoid multiple roscore errors.
add_rostest(test/pomdp_run_tiger_impatient.test)
add_rostest(test/pomdp_run_tiger_cautious.test)
add_rostest(test/pomdp_run_4x3_fast.test)
add_rostest(test/pomdp_run_4x3_fast_empty.test)
add_rostest(test/pomdp_run_4x4_compound_state_space.test)
add_rostest(test/oracle_irl_tiger.test)
add_rostest(test/dialogue_irl_tiger.test)

