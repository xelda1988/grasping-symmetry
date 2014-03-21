#ifndef GRASPSYMMETRIZER_GRASP_H_
#define GRASPSYMMETRIZER_GRASP_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
#include "gripper.h"
#include "../tinyxml2/tinyxml2.h"

namespace graspsym{
    
  class Grasp {
    
  public:
      
    std::string gripperIdentifier_; //name of gripper
    std::string objectIdentifier_;
    int dof_; //Degree of freedom  
    std::vector<PoseMat> handPoses_; //Pose of handframe
    std::vector<Eigen::VectorXf> jointPositions_;//Joint Positions (Length not known at compiletime)
    
    
    void loadFromGraspItXml(const std::string filePath); //Pointer to Element of XMLfile
    void saveToGraspItXml(const std::string layoutFilePath, const std::string saveFilePath);  //Needs preconfigured GraspItXml!
    void setGripper(const Gripper & gripper); //sets Dof and gripper Identifier
    
    void loadFromXml(tinyxml2::XMLElement* graspElement);
    void addToXml(std::string filePath);
    
    void extrapolateGraspSeq(); //extrapolates grasp Sequence linearly and or along approach direction
    
    void printGrasp() const;
    
    //Constructors
  //   Grasp(tinyxml2::XMLElement* graspElement,const Gripper &  gripper) {
  //     setGripper(gripper); //to check DOF!
  //     loadFromXml(graspElement);
  //   }
    
    Grasp(tinyxml2::XMLElement* graspElement)
    : dof_(7) //for now hardcoded
    {
      loadFromXml(graspElement);
    }
    
    Grasp(const std::string filePath) {
      loadFromGraspItXml(filePath);
    }
    
    //Grasp();
    //destructor
    //~Grasp();
  };

  class GraspDatabase {
    
    
  public:
    
    std::vector<Grasp> graspDb_;
    void loadFromXml(const std::string filePath);
    void push_back(const std::vector<Grasp> & graspDb){
    graspDb_.insert(graspDb_.end(), graspDb.begin(), graspDb.end() );
    }
    void push_back(const Grasp & grasp){
      graspDb_.push_back(grasp);
    }
    void printGraspDatabase();
    void saveToXml(const std::string filePath); //TODO
    GraspDatabase(const std::string filePath) {
      loadFromXml(filePath); //Pass gripper through constructor
    }
    GraspDatabase(std::vector<Grasp> graspDb) {
      graspDb_=graspDb;//Pass gripper through constructor
    }
    
    //~GraspDatabase();
  };
}//NS

#endif