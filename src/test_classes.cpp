#include <Eigen/Dense>
#include "GraspSymmetrizer/defs.h"
#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/gripper.h"
#include "GraspSymmetrizer/object.h"
#include <iostream>
#include "GraspSymmetrizer/symmetry.h"
#include "GraspSymmetrizer/util.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

int main () {
 
  //Do Something
  
  
  
  
  
  
  
  
  
  
  
  
  /*
  //Test util done:
  
  /////////////
  cout << "Testing util.cpp" << endl;  
  const char* numberstring= "1.56";
  const char* numberstring2= "1";
  float numberfloat = 0.4;
  int numberint = 4;
  
  cout << "String: " << CharArrayToString(numberstring) << endl;
  cout << "Float: " << CharArrayToFloat(numberstring2) << endl;
  cout << "Int: " << CharArrayToInt(numberstring) << endl;
  
  cout << "String of Float: " << FloatToString(numberfloat) << endl;
  cout << "String of Int: " << IntToString(numberint) << endl;
  /////////////
  
  PoseMat poseMat = Matrix4f::Identity();
  poseMat(1,3) = 0.4f;
  PoseQuat poseQuat;
  PoseEuler poseEuler;
  
  cout << "PoseMatStart: \n" << poseMat << endl;
  
  PoseMatToPoseEuler(poseEuler, poseMat);
  PoseMatToPoseQuat(poseQuat, poseMat);
  
  cout << "PoseEuler: " << poseEuler.transpose() << endl;
  cout << "PoseQuat: " << poseQuat.transpose() << endl;
  
  PoseQuatToPoseMat(poseMat, poseQuat);
  PoseQuatToPoseEuler(poseEuler, poseQuat);
  
  cout << "PoseMat: \n" << poseMat << endl;
  cout << "PoseEuler: " << poseEuler.transpose() << endl;
  
  PoseEulerToPoseMat(poseMat, poseEuler);
  PoseEulerToPoseQuat(poseQuat, poseEuler);
  
  cout << "PoseMat: \n" << poseMat << endl;
  cout << "PoseQuat: " << poseQuat.transpose() << endl;
  
  /////////////
  XMLDocument doc;
  doc.LoadFile("../resources/SchunkDexHandConfig.xml");
  
  XMLElement * xeManip= doc.FirstChildElement()->FirstChildElement("manipulator");
  vector<string> attNames, attNames2;
  vector<string> attValues;
  vector<float> attValuesFloat;
  vector<int> attValuesInt;
  
  attNames.push_back("name");
  attNames.push_back("dof");
  attNames2.push_back("x");
  attNames2.push_back("y");
  attNames2.push_back("z");
  attNames2.push_back("roll");
  attNames2.push_back("pitch");
  attNames2.push_back("yaw");
  
  getAttributeList(attValues, attNames, xeManip);
  
  cout << "Attribute Values" << endl;
  for(int i = 0; i < attValues.size(); i++)
    cout << attValues.at(i) << endl;
  
  XMLElement * xeGlobalPose = xeManip->FirstChildElement("global_pose");
  
  
  getAttributeList(attValuesFloat, attNames2, xeGlobalPose);
  getAttributeList(attValuesInt, attNames2, xeGlobalPose);
  cout << "Attribute ValuesFloat" << endl;
  for(int i = 0; i < attValuesFloat.size(); i++)
    cout << attValuesFloat.at(i) << endl;
  cout << "Attribute ValuesInt" << endl;
  for(int i = 0; i < attValuesInt.size(); i++)
    cout << attValuesInt.at(i) << endl;
  */
  
  
  cout << "Classes Succesfully Tested! - Cheers" << endl;
  return 0;
}