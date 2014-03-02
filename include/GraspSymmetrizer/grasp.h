#ifndef GRASPSYMMETRIZER_GRASP_H_
#define GRASPSYMMETRIZER_GRASP_H_

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
  std::vector<PoseMat> handPose_; //Pose of handframe
  std::vector<Eigen::MatrixXf> jointPositions_;//Joint Positions (Length not known at compiletime)
  
public:
  
  void loadFromGraspItXml(const std::string filePath); //Pointer to Element of XMLfile
  void saveToGraspItXml(const std::string filePath);  //Needs preconfigured GraspItXml!
  void setGripper(const Gripper & gripper); //sets Dof and gripper Identifier
  
  void loadFromXml(tinyxml2::XMLElement* graspElement);
  void addToXml(tinyxml2::XMLElement* graspElement);
  
  void printGrasp();
  
  //Constructors
  Grasp(tinyxml2::XMLElement* graspElement,const Gripper &  gripper) {
    loadFromXml(graspElement);
    setGripper(gripper);
  }
  
  Grasp(const std::string filePath, const Gripper & gripper) {
    loadFromGraspItXml(filePath);
    setGripper(gripper);
  }
  //destructor
  ~Grasp();
};

class GraspDatabase {
  
  std::vector<Grasp> graspDb_;
  
public:
  
  void loadFromXml(const std::string filePath, const Gripper & gripper);
  void printGraspDatabase();
  
  GraspDatabase(const std::string filePath, const Gripper & gripper) {
    loadFromXml(filePath, gripper); //Pass gripper through constructor
  }
  
  ~GraspDatabase();
};

#endif