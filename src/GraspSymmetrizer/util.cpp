#include "GraspSymmetrizer/util.h"
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <limits>

//Basic Type Conversions  
int charArrayToInt(const char* charPtr){
  
  int x;
  try
  {
    x = boost::lexical_cast<int>(charPtr);  
  }  
  catch(const boost::bad_lexical_cast &)
  {
    std::cout << "[Exception] Casting to Integer failed, trying to round from float" << std::endl;
    x = (int) boost::lexical_cast<float>(charPtr); 
  }
  return x;
}

std::string CharArrayToString(const char* charPtr){
 std::string s(charPtr);
 return s; 
}

float charArrayToFloat(const char* charPtr){
  float x;
  try
  {
    x = boost::lexical_cast<float>(charPtr);  
  }  
  catch(const boost::bad_lexical_cast &)
  {
    std::cout << "[Exception] Casting to Float failed! \n Exit" << std::endl;
    exit(EXIT_FAILURE);
  }
  return x;
}

std::string intToString(const int number){
  std::stringstream ss;
  ss << number;
  return ss.str();
}
  
std::string floatToString(const float number){
  std::stringstream ss;
  ss << number;
  return ss.str();
}

//Pose Conversions  
void poseEulerToPoseMat(PoseMat & poseMat, const PoseEuler & poseEuler){
  
  Eigen::Matrix3f rot;
  PoseMat posematrix;
  
  rot = Eigen::AngleAxisf(poseEuler(3), Eigen::Vector3f::UnitZ())* Eigen::AngleAxisf(poseEuler(4), Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(poseEuler(5), Eigen::Vector3f::UnitX()); //euler angles
  posematrix.col(0) << rot.col(0),0;
  posematrix.col(1) << rot.col(1),0;
  posematrix.col(2) << rot.col(2),0;
  posematrix.col(3) << poseEuler(0),poseEuler(1),poseEuler(2),1;

  poseMat = posematrix;
}
   
void poseEulerToPoseQuat(PoseQuat & poseQuat, const PoseEuler & poseEuler){
  
  PoseMat poseMat;
  
  poseQuat.head(3)=poseEuler.head(3);
  poseEulerToPoseMat(poseMat, poseEuler);
  Eigen::Quaternionf quat(poseMat.block<3,3>(0,0));
  
  poseQuat(3)=quat.w();
  poseQuat(4)=quat.x();
  poseQuat(5)=quat.y();
  poseQuat(6)=quat.z(); 
}

void poseMatToPoseEuler(PoseEuler & poseEuler, const PoseMat & poseMat){
  
  PoseEuler pose;
  Eigen::Vector3f trans, angles;
  Eigen::Matrix3f rot;
  
  rot.col(0) << poseMat.col(0).head(3);
  rot.col(1) << poseMat.col(1).head(3);
  rot.col(2) << poseMat.col(2).head(3);
  trans << poseMat.col(3).head(3);
  angles = rot.eulerAngles(2, 1, 0);
  pose << trans, angles;
  
  poseEuler = pose;
}
   
void poseMatToPoseQuat(PoseQuat & poseQuat, const PoseMat & poseMat){
  
  Eigen::Quaternionf quat(poseMat.block<3,3>(0,0));
  
  poseQuat.head(3)=poseMat.col(3).head(3);
  poseQuat(3)=quat.w();
  poseQuat(4)=quat.x();
  poseQuat(5)=quat.y();
  poseQuat(6)=quat.z(); 
}
void poseQuatToPoseMat(PoseMat & poseMat, const PoseQuat & poseQuat){
  
  Eigen::Vector3f trans;
  Eigen::Quaternionf quat;
  Eigen::Matrix3f rot;
  PoseMat posematrix;
  
  trans = poseQuat.head(3);
  quat.w()=poseQuat(3);
  quat.x()=poseQuat(4);
  quat.y()=poseQuat(5);
  quat.z()=poseQuat(6);
  rot=quat.toRotationMatrix();
  
  posematrix.col(0) << rot.col(0),0;
  posematrix.col(1) << rot.col(1),0;
  posematrix.col(2) << rot.col(2),0;
  posematrix.col(3) << trans(0),trans(1),trans(2),1; 
  
  poseMat = posematrix;
}
//   
void poseQuatToPoseEuler(PoseEuler & poseEuler, const PoseQuat & poseQuat){
  PoseMat poseMat;
  poseQuatToPoseMat(poseMat, poseQuat);
  poseMatToPoseEuler(poseEuler, poseMat);  
}
//   
//   //getAttributesFrom XML Element
//For floats also nan is defined
void getAttributeList(  std::vector<float> & floatList, const std::vector<std::string> & attributeNames,  const tinyxml2::XMLElement* xmlElementPtr){
  try
  { 
    float nan  = std::numeric_limits<float>::quiet_NaN();
    
    for(int i = 0; i < attributeNames.size(); i++)
    {
      
      const char* actualCharArray = xmlElementPtr->Attribute( attributeNames.at(i).c_str() );
      std::string actualString = CharArrayToString(actualCharArray);
      if (actualString == "NaN") floatList.push_back( nan );
      else {
	float actualFloat = xmlElementPtr->FloatAttribute( attributeNames.at(i).c_str() );
	floatList.push_back( actualFloat );
      }      
    }
  }
  catch(std::exception &e)
  {
    std::cout << "[Exception] getAttributeList(std::vector<float>): Names seem not to match the XML attributes, exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
  
}

void getAttributeList(  std::vector<int> & intList, const std::vector<std::string> & attributeNames, const tinyxml2::XMLElement* xmlElementPtr){
  try
  {    
    for(int i = 0; i < attributeNames.size(); i++)
    {
      float actualInt = xmlElementPtr->IntAttribute( attributeNames.at(i).c_str() );
      intList.push_back( actualInt );
    }
  }
  catch(std::exception &e)
  {
    std::cout << "[Exception] getAttributeList(std::vector<int>): Names seem not to match the XML attributes, exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void getAttributeList(  std::vector<std::string> & stringList, const std::vector<std::string> & attributeNames, const tinyxml2::XMLElement* xmlElementPtr){
  try
  {
    for(int i = 0; i < attributeNames.size(); i++)
    {
      const char* actualCharArray = xmlElementPtr->Attribute( attributeNames.at(i).c_str() );
      stringList.push_back( CharArrayToString(actualCharArray) );
    }
  }
  catch(std::exception &e)
  {
    std::cout << "[Exception] getAttributeList (std::vector<string>): Names seem not to match the XML attributes, exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void getSymmetryTypeFromString(SymmetryType & symmetryType, const std::string symmetryTypeStr){
 
  if(symmetryTypeStr == "singleplane") symmetryType=SINGLEPLANE;
  else if (symmetryTypeStr == "doubleplane") symmetryType=DOUBLEPLANE;
  else if (symmetryTypeStr == "tripleplane") symmetryType=TRIPLEPLANE;
  else if (symmetryTypeStr == "axial") symmetryType=AXIAL;
  else if (symmetryTypeStr == "axialsingleplane") symmetryType=AXIALSINGLEPLANE;
  else if (symmetryTypeStr == "c3") symmetryType=C3;
  else 
  {std::cout << "Objectsymmetry not correctly defined, exiting!\n";
    exit(EXIT_FAILURE);
  }
}

void getConstraintTypeFromString(ConstraintType & constraintType, const std::string constrainTypeStr){
 
  if(constrainTypeStr == "bool") constraintType=BOOL;
  else if (constrainTypeStr == "continuous") constraintType=CONTINUOUS;
  else 
  {std::cout << "Constraint Symmetry not correctly defined, exiting!\n";
    exit(EXIT_FAILURE);
  }
}


