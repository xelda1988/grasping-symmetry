#ifndef GRASPSYMMETRIZER_OBJECT_H_
#define GRASPSYMMETRIZER_OBJECT_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
#include "gripper.h"
#include "../tinyxml2/tinyxml2.h"


class Object {
  
  int objecID_;
  std::string objectName_;
  ObjectSymmetry objectSymmetry_; //also data definition
  
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