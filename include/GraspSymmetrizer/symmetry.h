#ifndef GRASPSYMMETRIZER_SYMMETRY_H_
#define GRASPSYMMETRIZER_SYMMETRY_H_

#include <vector>
#include "defs.h"
#include "gripper.h"
#include "object.h"
#include "grasp.h"

//symmetryOperators:
void pointReflectionMatrix(Eigen::Matrix4f & reflectionMat, const Point3D point); //Implement TODO LATER
void reflectionMatrix(Eigen::Matrix4f & reflectionMat, const Layer3D layer); //Point reflection Operator given layer
void axisRotationMatrix(Eigen::Matrix4f & axisMat, const Axis3D axis, const float alpha); //thumb rule, where point1 is the top, right-handed rotation
void axisRotationMatrixesN(std::vector<Eigen::Matrix4f> & axisMatrixes, const Axis3D axis, const float alpha, const int nrRotations); //subsequent single rotation, gives back all but identity rotation
void getLayerFromSymAxis(Layer3D & layer, const Axis3D & symAxis);

class SymmetryOperation {
 
  GripperSymmetry gripperSymmetry_;
  ObjectSymmetry objectSymmetry_;
  int rotationSamplingNr_;
  
  std::vector<Grasp> resultingGrasps_;
  Grasp inputGrasp_;
  std::vector<int> jointAndPoseAssociation_;
  
  void reflectGrasps(const Layer3D & objectLayer, const Layer3D & gripperLayer, std::vector<Grasp> & grasp_list, const std::vector<Eigen::VectorXi> & jointConstraints); //does Operation on resultingGrasps
  void rotateGrasps(const Axis3D axis, std::vector<Grasp> & grasp_list);
  void flipJointConstraintBool(const std::vector<Eigen::VectorXi> & jointConstraints, std::vector<Eigen::VectorXf> & jointConfigurations );
  bool checkConstraintContinuous(const std::vector<Eigen::VectorXf> & jointConstraints, const std::vector<Eigen::VectorXf> & jointConfigurations );
public:
  
  void getActiveGripperSymmetry(const Grasp & grasp, SymmetryType symType); //for Current Grasp
  void checkSetInputGrasp(const Gripper & gripper, const Object & object);
  void computeSymmetries();
    GraspDatabase getGraspDb(){
    GraspDatabase graspDb(resultingGrasps_);
    return graspDb;
  }
  
  SymmetryOperation(const Gripper & gripper, const Object & object, const Grasp & grasp)
  : inputGrasp_(grasp),rotationSamplingNr_(10)
  {
    gripperSymmetry_ = gripper.gripperSymmetry_.at(0); //TODO Here one should have a function to check which symmetry applies for the current grasp!
    objectSymmetry_ = object.getObjectSymmetry();
    checkSetInputGrasp(gripper, object);
    
  }
//   ~SymmetryOperation();
};

#endif