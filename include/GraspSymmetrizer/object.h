#ifndef OBJECT_H_
#define OBJECT_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
#include "gripper.h"
#include "../tinyxml2/tinyxml2.h"


class Object {
  
  int objecID_;
  std::string objectName_;
  
   
  int dof_; //Degree of freedom  
  Pose globalPose_; //Global Pose offset of handframe, maybe needed later
  Eigen::MatrixXf jointPositions_; //Current Gripper Joint Position
  
  //Symmetrystates
  struct Constraint {
   std::string type
   std::vector<Eigen::MatrixXf> jointConfigurations;
  }; //Here only type definition
  
  struct Symmetry {
    std::vector<Constraint> symmmetries;
  } Symmetry_; //also data definition
  
public:
  
  loadFromXml(std::string filePath); //Pointer to Element of XMLfile
    
  //Constructors
  Gripper(std::string filePath, Gripper gripper) {
    loadFromXml(std::string filePath);
  }
  //destructor
  ~Gripper();
};

#endif