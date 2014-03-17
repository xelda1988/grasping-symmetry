#ifndef GRASPSYMMETRIZER_GRIPPER_H_
#define GRASPSYMMETRIZER_GRIPPER_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
//#include "gripper.h"
#include "../tinyxml2/tinyxml2.h"


class Gripper {
  
  std::string gripperIdentifier_; //name of gripper
   
  int dof_; //Degree of freedom  
  PoseMat globalPose_; //Global Pose offset of handframe, maybe needed later
  Eigen::VectorXf jointPositions_; //Current Gripper Joint Position
  
  
  
public:
  
  std::vector<GripperSymmetry> gripperSymmetry_; //also data definition
  void loadFromXml(std::string filePath); //Pointer to Element of XMLfile
  void printGripperInfo();
  std::string getName() const;
  int getDof() const;
  //Constructors
  Gripper(std::string filePath) {
    loadFromXml(filePath);
  }
  //destructor
  //~Gripper();
};


#endif