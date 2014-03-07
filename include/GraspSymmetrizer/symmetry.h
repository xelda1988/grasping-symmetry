#ifndef GRASPSYMMETRIZER_SYMMETRY_H_
#define GRASPSYMMETRIZER_SYMMETRY_H_

#include <vector>
#include "defs.h"
#include "gripper.h"
#include "object.h"
#include "grasp.h"

//symmetryOperators:
void pointReflectionMatrix(Eigen::Matrix4f & reflectionMat, const Point3D point);
void reflectionMatrix(Eigen::Matrix4f & reflectionMat, const Layer3D layer); //Point reflection Operator given layer
void axisRotationMatrix(Eigen::Matrix4f & axisMat, const Axis3D axis, const float alpha); //thumb rule, where point1 is the top, right-handed rotation
void axisRotationMatrixesN(std::vector<Eigen::Matrix4f> & axisMatrixes, const Axis3D axis, const float alpha, const int nrRotations); //subsequent single rotation, gives back all but identity rotation

class SymmetryOperation {
 
  GripperSymmetry gripperSymmetry_;
  ObjectSymmetry objectSymmetry_;
  int rotationSamplingNr_;
  
  std::vector<Grasp> resultingGrasps_;
  Grasp inputGrasp_;
  
  void reflectGrasps(const Layer3D layer); //does Operation on resultingGrasps
  void rotateGrasps(const Axis3D axis);
public:
  
  void computeSymmetries();
  void getGraspDb(GraspDatabase & graspDb);
  
  SymmetryOperation(const Gripper & gripper, const Object & object, const Grasp & grasp)
  : inputGrasp_(grasp)
  {
    //set variables ...
  }
  ~SymmetryOperation();
};

#endif