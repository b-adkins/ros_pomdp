cmake_minimum_required(VERSION 2.8.3)
project(pomdp)

set(ROS_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rostest)

##System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# Path to A. Cassandra's pomdp-solve headers
# @todo Work this into a more elegant place, e.g. rospack.
set(LIB_POMDP_SOLVE_INCLUDE_DIR ../../../pomdp-solve-5.3/src) 

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${LIB_POMDP_SOLVE_INCLUDE_DIR}
  LIBRARIES pomdp
  CATKIN_DEPENDS roscpp #rostest
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(src include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIB_POMDP_SOLVE_INCLUDE_DIR}
)

set(POMDP_FILES src/pretty.cpp src/mdp.cpp src/pomdp.cpp src/mdp_factory.cpp src/mdp_util.c src/empty_mdp.cpp 
                src/pomdp_io.cpp)
set(POLICY_FILES src/pretty.cpp src/policy.cpp src/policy_factory.cpp src/map_policy.cpp src/alpha_policy.cpp 
                 src/multinomial_policy.cpp src/pomdp_io.cpp)
set(FEATURE_FILES src/simple_feature_value.cpp src/belief_feature_value.cpp src/feature.cpp src/feature_vector.cpp src/simple_feature_space.cpp src/vector_feature_space.cpp src/feature_space_factory.cpp)

# Libraries necessary for GNU Science Library
set(GSL_LIBRARIES gsl gslcblas m)

## Declare a cpp library
add_library(pomdp STATIC
  ${POMDP_FILES} ${POLICY_FILES} ${FEATURE_FILES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pomdp_node pomdp_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pomdp
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   mdp
   ${GSL_LIBRARIES} # TODO Refactor this dependency to stochastic package out
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
# install(TARGETS pomdp pomdp_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pomdp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
catkin_add_nosetests(test)

# Manually add Google Tests (C++ unit tests).
catkin_add_gtest(test_map_policy test/test_map_policy.cpp)
catkin_add_gtest(test_alpha_policy test/test_alpha_policy.cpp)
catkin_add_gtest(test_pomdp test/test_pomdp.cpp)
catkin_add_gtest(test_pretty_print test/test_pretty.cpp)
catkin_add_gtest(test_stochastic_policies test/test_stochastic_policies)
catkin_add_gtest(test_feature_space test/test_feature_space.cpp)
target_link_libraries(test_map_policy ${catkin_LIBRARIES} pomdp yaml-cpp)
target_link_libraries(test_alpha_policy ${catkin_LIBRARIES} pomdp yaml-cpp)
target_link_libraries(test_pomdp ${catkin_LIBRARIES} pomdp yaml-cpp)
target_link_libraries(test_pretty_print ${catkin_LIBRARIES} pomdp yaml-cpp)
target_link_libraries(test_stochastic_policies ${catkin_LIBRARIES} ${GSL_LIBRARIES} pomdp yaml-cpp)
target_link_libraries(test_feature_space ${catkin_LIBRARIES} pomdp yaml-cpp)

## Build properties
# For targets that use c++0x features.
set_target_properties(test_feature_space PROPERTIES COMPILE_FLAGS -std=c++0x)

