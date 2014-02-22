# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Example Interface
PROJECT(GraspSymmetrizer)

# External Libraries

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/ros/groovy/stacks/arm_navigation/constraint_aware_spline_smoother/cmake")
FIND_PACKAGE(Eigen REQUIRED)
find_library(TINY_XML2 NAMES tinyxml2 PATHS ./include/tinyxml2/build)

# Settings
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(./include)

# Sources
SET(INSTALL_DIR ./bin)
SET(XMLTEST_SOURCES ./src/xml_test.cpp)
ADD_EXECUTABLE(../bin/xml_test ${XMLTEST_SOURCES})
TARGET_LINK_LIBRARIES(../bin/xml_test ${Eigen_LIBRARIES} ${TINY_XML2} )
#INSTALL(TARGETS xml_test DESTINATION ${CMAKE_BINARY_DIR}/../bin	 )