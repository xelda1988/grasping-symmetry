# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Example Interface
PROJECT(GraspSymmetrizer)

# External Libraries

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 

FIND_PACKAGE(Eigen)
FIND_PACKAGE(Boost)
find_library(TINY_XML2 NAMES tinyxml2 PATHS ./include/tinyxml2/build)

# Settings
#message("Eigen_Dirs:" ${Eigen_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(./include)

# Sources
SET(INSTALL_DIR ./bin)
SET(USRPATH ./src/GraspSymmetrizer)
SET(XMLTEST_SOURCES ./src/xml_test.cpp)

SET( CLASSTEST_SOURCES 
       
      ${USRPATH}/grasp.cpp 
      ${USRPATH}/object.cpp 
      ${USRPATH}/symmetry.cpp 
      ${USRPATH}/util.cpp 
      ${USRPATH}/gripper.cpp
)
  
ADD_EXECUTABLE(../bin/xml_test ${XMLTEST_SOURCES})
ADD_EXECUTABLE(test_classes ./src/test_classes.cpp ${CLASSTEST_SOURCES})
ADD_EXECUTABLE(graspit_batch_export ./src/graspit_batch_export.cpp ${CLASSTEST_SOURCES})

TARGET_LINK_LIBRARIES(../bin/xml_test ${Eigen_LIBRARIES} ${TINY_XML2} )
TARGET_LINK_LIBRARIES(test_classes ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${TINY_XML2} )
TARGET_LINK_LIBRARIES(graspit_batch_export ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${TINY_XML2} )