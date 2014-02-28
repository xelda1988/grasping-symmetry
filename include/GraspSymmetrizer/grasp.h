#ifndef GRASP_H_
#define GRASP_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
#include "gripper.h"
#include "../tinyxml2/tinyxml2.h"


class Grasp {
  
  std::string gripperIdentifier_; //name of gripper
  std::string objectIdentifier_;
  
  int dof_; //Degree of freedom  
  std::vector<Pose> handPose_; //Pose of handframe
  std::vector<Eigen::MatrixXf> jointPositions_;//Joint Positions (Length not known at compiletime)
  
public:
  
  loadFromGraspItXml(std::string filePath); //Pointer to Element of XMLfile
  saveToGraspItXml(std::string filePath);  //Needs preconfigured GraspItXml!
  setGripper(const Gripper & gripper); //sets Dof and gripper Identifier
  
  loadFromXml(tinyxml2::XMLElement* graspElement);
  addToXml(tinyxml2::XMLElement* graspElement);
  
  //Constructors
  Grasp(tinyxml2::XMLElement* graspElement, Gripper gripper) {
    loadFromXml(graspElement);
    setGripper(gripper);
  }
  
  Grasp(std::string filePath, const Gripper & gripper) {
    loadFromGraspItXml(filePath);
    setGripper(gripper);
  }
  //destructor
  ~Grasp();
};

#endif