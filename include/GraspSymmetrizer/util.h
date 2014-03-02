#ifndef GRASPSYMMETRIZER_UTIL_H_
#define GRASPSYMMETRIZER_UTIL_H_

//TODO: transform_link (offset, a,b) ?

#include <string>
#include <vector>
#include "defs.h"
#include "../tinyxml2/tinyxml2.h"

//namespace GraspSymmetrizer{
  
  //Basic Type Conversions  
  int CharArrayToInt(const char* charPtr);

  std::string CharArrayToString(const char* charPtr);

  float CharArrayToFloat(const char* charPtr);
  
  std::string IntToString(int number);
  
  std::string FloatToString(float number);

  //Pose Conversions
  void PoseEulerToPoseMat(PoseMat & poseMat, const PoseEuler & poseEuler);
  
  void PoseEulerToPoseQuat(PoseQuat & poseQuat, const PoseEuler & poseEuler);
  
  void PoseMatToPoseEuler(PoseEuler & poseEuler, const PoseMat & poseMat);
  
  void PoseMatToPoseQuat(PoseQuat & poseQuat, const PoseMat & poseMat);
  
  void PoseQuatToPoseMat(PoseMat & poseMat, const PoseQuat & poseQuat);
  
  void PoseQuatToPoseEuler(PoseEuler & poseEuler, const PoseQuat & poseQuat);
  
  //getAttributesFrom XML Element
  void getAttributeList(  std::vector<float> & floatList, const std::vector<std::string> & AttributeNameList,  const tinyxml2::XMLElement* xmlElementPtr);
  void getAttributeList(  std::vector<int> & intList, const std::vector<std::string> & AttributeNameList, const tinyxml2::XMLElement* xmlElementPtr);
  void getAttributeList(  std::vector<std::string> & stringList, const std::vector<std::string> & AttributeNameList, const tinyxml2::XMLElement* xmlElementPtr);

//}

#endif