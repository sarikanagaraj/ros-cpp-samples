cmake_minimum_required(VERSION 2.8.3)
project(robone_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
     CATKIN_DEPENDS geometry_msgs roscpp tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable

# Added header files for IDE to see those files
set(SLAM_OPTIMIZER_HEADER_FILES
  include/algorithm/coordinate_descent.h
  include/exception/application_error.h
  include/function/navigation_quality_function.h
  include/function/parameter.h
  include/function/single_value_function.h
  include/function/multi_value_function.h
  include/util/string_utils.h
  include/yaml/yaml_config.h
)

set(SLAM_OPTIMIZER_SRC_FILES 
  src/function/navigation_quality_function.cc
  src/function/parameter.cc
  src/util/string_utils.cc
  src/yaml/yaml_config.cc
)

add_executable(slam_optimizer 
    src/application/slam_optimizer.cc
    ${SLAM_OPTIMIZER_HEADER_FILES}
    ${SLAM_OPTIMIZER_SRC_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(slam_optimizer
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  include_directories(
    test/include
  )

  catkin_add_gtest(slam-optimizer-test 
    test/src/application/slam_optimizer_test.cc
    test/src/suite/coordinate_descent_test.cc
    test/src/suite/string_utils_test.cc
    test/src/suite/yaml_config_test.cc
    test/include/function/paraboloid_function.h
    test/src/function/paraboloid_function.cc
    test/include/util/temp_file.h
    test/src/util/temp_file.cc
    ${SLAM_OPTIMIZER_HEADER_FILES}
    ${SLAM_OPTIMIZER_SRC_FILES}
  )

  if(TARGET slam-optimizer-test)
    target_link_libraries(slam-optimizer-test  ${catkin_LIBRARIES})
  endif()

endif()
