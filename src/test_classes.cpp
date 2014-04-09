#include <iostream>
#include <Eigen/Dense>
#include "GraspSymmetrizer/defs.h"
#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/gripper.h"
#include "GraspSymmetrizer/object.h"
#include "GraspSymmetrizer/symmetry.h"
#include "GraspSymmetrizer/util.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;
using namespace graspsym;

int main () {
 
  //Do Something
  
  
  
  
  
  
  //Test symmetry.cpp
  
  //xy Layer
  
//   Layer3D xyPlane;
//   xyPlane.point1 << 0,0,0;
//   xyPlane.point2 << 3,0,0;
//   xyPlane.point3 << 0,2,0;
//   
//   Axis3D zAxis;
//   zAxis.point1 << 0,0,1;
//   zAxis.point2 << 0,0,-1;
//   
//   Matrix4f reflectMatrix, axisRotMat;
//   
//   reflectionMatrix(reflectMatrix, xyPlane);
//   axisRotationMatrix(axisRotMat, zAxis, M_PI / 2.0f); //rotates in + direction
//   
//   
//   cout << "ReflectionMatrix xy plane: \n" << reflectMatrix << endl;
//   cout << "Rotation zAxis \n" << axisRotMat << endl;
//   
//   Vector4f testPoint, testPoint2;
//   
//   testPoint << 0,0,1,0;
//   testPoint2 << 1,0,0,0;
//   
//   cout << "Reflected Point \n" << reflectMatrix*testPoint << endl;
//   cout << "Rotated Point \n" << axisRotMat*testPoint2 << endl;
//
  
  
  //Load a single grasp from graspIt:
//   const char* layoutpath="/home/alexander/workspace/GraspSymmetrizer/resources/graspit/faceplate_1_grasp_graspit.xml";
//   const char* savepath = "/home/alexander/workspace/GraspSymmetrizer/resources/graspit/fp_mirrored.xml";
//  
//   Grasp graspItgrasp(layoutpath);
//   //Test gripper.cpp done
// //    graspItgrasp.saveToGraspItXml(layoutpath, savepath );
//   
//   Gripper gripper("/home/alexander/workspace/GraspSymmetrizer/resources/SchunkDexHandConfig.xml");
// //   gripperInstance.printGripperInfo();
//   
// //   GraspDatabase graspDb("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_base_layout.xml");
// //    graspDb.printGraspDatabase();
//   
//   ObjectDatabase objectDb("../resources/ObjectDatabase.xml");
// //   objectDb.printObjectDatabase();
//   
//   SymmetryOperation symOp(gripper, objectDb.objectDb_.at(0), graspItgrasp );
//   //z axis
//   symOp.computeSymmetries();
//   GraspDatabase graspDbOut = symOp.getGraspDb();
//   
//    graspDbOut.printGraspDatabase();
//    graspDbOut.saveToXml("/home/alexander/workspace/GraspSymmetrizer/resources/graspDb_gen.xml");
// //    graspDbOut.graspDb_.at(1).saveToGraspItXml(layoutpath, savepath );
//   /*
//   
//   //Test grasp.cpp 
//   
//   XMLDocument doc;
//   doc.LoadFile("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_base_layout.xml");
//   
//   XMLElement * graspPtr = doc.FirstChildElement()->FirstChildElement("grasp");
//   Grasp grasp(graspPtr);
//   grasp.printGrasp();
//   
//   cout << "Testing grapIt loader: " << endl;
//   
//   Grasp grasp2("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_GraspIt_import.xml");
//   grasp2.printGrasp();
//   
//   //grasp.addToXml("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_base_layout_write.xml");
//   
//   
//   GraspDatabase graspDb("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_base_layout_write.xml");
//   graspDb.printGraspDatabase();
//   
//   //Test gripper.cpp done
//   
//   Gripper gripperInstance("/home/alexander/workspace/GraspSymmetrizer/resources/SchunkDexHandConfig.xml");
//   gripperInstance.printGripperInfo();
//   
//   ////////////////////
//   //Test object.cpp done
//   
//   XMLDocument doc;
//   doc.LoadFile("../resources/ObjectDatabase.xml");
//   
//   XMLElement * xeFaceplate= doc.FirstChildElement()->FirstChildElement("object");
//   
//   Object faceplate(xeFaceplate);
//   
//   faceplate.printObjectInfo();
//   
//   ObjectDatabase objectDb("../resources/ObjectDatabase.xml");
//   
//   objectDb.printObjectDatabase();
//   
//   //Test util done:
  
//   /////////////
//   cout << "Testing util.cpp" << endl;  
//   const char* numberstring= "1.56";
//   const char* numberstring2= "1";
//   float numberfloat = 0.4;
//   int numberint = 4;
//   
//   cout << "String: " << CharArrayToString(numberstring) << endl;
//   cout << "Float: " << CharArrayToFloat(numberstring2) << endl;
//   cout << "Int: " << CharArrayToInt(numberstring) << endl;
//   
//   cout << "String of Float: " << FloatToString(numberfloat) << endl;
//   cout << "String of Int: " << IntToString(numberint) << endl;
//   /////////////
//   
  
  
  PoseEuler poseUr5;
  
  poseUr5 << 0.05648, 0.04456, 0.4365, 1.9357, 0.1020, 0.5554;
  
  PoseQuat poseMoveIt;
  
  poseMoveIt << 0.511514, -0.071577, 1.48908, 0.581692, 0.0113508, -0.766832, 0.271061;
  
  PoseMat Ur5;
  PoseMat Moveit;
  PoseMat Ur5_to_Moveit;
  PoseQuat Ur5_to_Moveit_Quat;
  
  poseEulerToPoseMat(Ur5, poseUr5);
  poseQuatToPoseMat(Moveit, poseMoveIt);
  
  Ur5_to_Moveit = Moveit * Ur5.inverse();
  poseMatToPoseQuat (Ur5_to_Moveit_Quat,Ur5_to_Moveit );
  cout << "Ur5_to_Moveit: \n" << Ur5_to_Moveit << endl;
  cout << "Ur5_to_Moveit Quat xyzwxyz: \n" << Ur5_to_Moveit_Quat << endl;
  
  
//   poseMatToPoseEuler(poseEuler, poseMat);
//   poseMatToPoseQuat(poseQuat, poseMat);
//   
//   cout << "PoseEuler: " << poseEuler.transpose() << endl;
//   cout << "PoseQuat: " << poseQuat.transpose() << endl;
//   
//   poseQuatToPoseMat(poseMat, poseQuat);
//   poseQuatToPoseEuler(poseEuler, poseQuat);
//   
//   cout << "PoseMat: \n" << poseMat << endl;
//   cout << "PoseEuler: " << poseEuler.transpose() << endl;
//   
//   poseEulerToPoseMat(poseMat, poseEuler);
//   poseEulerToPoseQuat(poseQuat, poseEuler);
//   
//   cout << "PoseMat: \n" << poseMat << endl;
//   cout << "PoseQuat: " << poseQuat.transpose() << endl;
//   
//   /////////////
//   XMLDocument doc;
//   doc.LoadFile("../resources/SchunkDexHandConfig.xml");
//   
//   XMLElement * xeManip= doc.FirstChildElement()->FirstChildElement("manipulator");
//   vector<string> attNames, attNames2;
//   vector<string> attValues;
//   vector<float> attValuesFloat;
//   vector<int> attValuesInt;
//   
//   attNames.push_back("name");
//   attNames.push_back("dof");
//   attNames2.push_back("x");
//   attNames2.push_back("y");
//   attNames2.push_back("z");
//   attNames2.push_back("roll");
//   attNames2.push_back("pitch");
//   attNames2.push_back("yaw");
//   
//   getAttributeList(attValues, attNames, xeManip);
//   
//   cout << "Attribute Values" << endl;
//   for(int i = 0; i < attValues.size(); i++)
//     cout << attValues.at(i) << endl;
//   
//   XMLElement * xeGlobalPose = xeManip->FirstChildElement("global_pose");
//   
//   
//   getAttributeList(attValuesFloat, attNames2, xeGlobalPose);
//   getAttributeList(attValuesInt, attNames2, xeGlobalPose);
//   cout << "Attribute ValuesFloat" << endl;
//   for(int i = 0; i < attValuesFloat.size(); i++)
//     cout << attValuesFloat.at(i) << endl;
//   cout << "Attribute ValuesInt" << endl;
//   for(int i = 0; i < attValuesInt.size(); i++)
//     cout << attValuesInt.at(i) << endl;
//   */
//   
//   
//   cout << "Classes Succesfully Tested! - Cheers" << endl;
  return 0;
}