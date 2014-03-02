#include <Eigen/Dense>
#include "GraspSymmetrizer/defs.h"
#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/gripper.h"
#include "GraspSymmetrizer/object.h"
#include <iostream>
//#include "GraspSymmetrizer/symmetry.h"
#include "GraspSymmetrizer/util.h"

using namespace std;
int main () {
 
  //Do Something
  //Test util:
  
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
  
  PoseMat poseMat = Eigen::Matrix4f::Identity();
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
 
  cout << "Classes Succesfully Tested! - Cheers" << endl;
  return 0;
}