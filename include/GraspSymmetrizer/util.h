#ifndef GRASPSYMMETRIZER_UTIL_H_
#define GRASPSYMMETRIZER_UTIL_H_

//TODO: transform_link (offset, a,b) ?

#include <string>
#include <vector>
#include "defs.h"
#include "../tinyxml2/tinyxml2.h"

//namespace GraspSymmetrizer{
  
  //Basic Type Conversions  
  int charArrayToInt(const char* charPtr);

  std::string charArrayToString(const char* charPtr);

  float charArrayToFloat(const char* charPtr);
  
  std::string intToString(int number);
  
  std::string floatToString(float number);

  //Pose Conversions
  void poseEulerToPoseMat(PoseMat & poseMat, const PoseEuler & poseEuler);
  
  void poseEulerToPoseQuat(PoseQuat & poseQuat, const PoseEuler & poseEuler);
  
  void poseMatToPoseEuler(PoseEuler & poseEuler, const PoseMat & poseMat);
  
  void poseMatToPoseQuat(PoseQuat & poseQuat, const PoseMat & poseMat);
  
  void poseQuatToPoseMat(PoseMat & poseMat, const PoseQuat & poseQuat);
  
  void poseQuatToPoseEuler(PoseEuler & poseEuler, const PoseQuat & poseQuat);
  
  //getAttributesFrom XML Element
  void getAttributeList(  std::vector<float> & floatList, const std::vector<std::string> & AttributeNameList,  const tinyxml2::XMLElement* xmlElementPtr);
  void getAttributeList(  std::vector<int> & intList, const std::vector<std::string> & AttributeNameList, const tinyxml2::XMLElement* xmlElementPtr);
  void getAttributeList(  std::vector<std::string> & stringList, const std::vector<std::string> & AttributeNameList, const tinyxml2::XMLElement* xmlElementPtr);
  
  void getSymmetryTypeFromString(SymmetryType & symmetryType, const std::string symmetryTypeStr);
  void getConstraintTypeFromString(ConstraintType & constraintType, const std::string constrainTypeStr);

//}

#endif