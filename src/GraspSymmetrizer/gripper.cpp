#include "GraspSymmetrizer/gripper.h"
#include "GraspSymmetrizer/util.h"
#include <limits>

void Gripper::loadFromXml(std::string filePath){
 
  std::vector<std::string> attributesManipulator, 
			   attributesGlobalPose, 
			   attributesSymmetry,
			   attributesConstraint,
			   attributesJointConfiguration,
			   valuesManipulator,
			   valuesSymmetry,
			   valuesConstraint,
			   valuesJointConfigurationString; //map this to Int
			   
  std::vector<float> valuesGlobalPose, 
		     valuesJointConfigurationFloat;
  std::vector<int> valuesJointConfigurationInt;
		    
  attributesManipulator.push_back("name");	
  attributesManipulator.push_back("dof");
  
  attributesGlobalPose.push_back("x");
  attributesGlobalPose.push_back("y");
  attributesGlobalPose.push_back("z");
  attributesGlobalPose.push_back("roll");
  attributesGlobalPose.push_back("pitch");
  attributesGlobalPose.push_back("yaw");
  
  attributesSymmetry.push_back("type");
  attributesConstraint.push_back("type");
  
  //here Schunk hand attributes Hardcoded - TODO replace generic
  for(int i = 1; i < 8; i++){
    std::string jointName = "j" + intToString(i);
    attributesJointConfiguration.push_back(jointName);
  }
  
  
  tinyxml2::XMLDocument doc;
  doc.LoadFile(filePath.c_str());
  
  if (!doc.Error()) {
    
    //get Manipulator Attributes
    tinyxml2::XMLElement * xeManip = doc.FirstChildElement()->FirstChildElement("manipulator");
    tinyxml2::XMLElement * xeGlobalPose = xeManip->FirstChildElement("global_pose");
    tinyxml2::XMLElement * xeJointConfiguration = xeManip->FirstChildElement("joint_configuration");
    
    if (xeManip) 
    {
      getAttributeList(valuesManipulator, attributesManipulator, xeManip);
    }
    else
    {
      std::cout << "Pointer to XML Element Manipulator not correct" << std::endl;
      exit(EXIT_FAILURE);
    }
    
    gripperIdentifier_=valuesManipulator.at(0);
    dof_=charArrayToInt(valuesManipulator.at(1).c_str());
    
    //get attributesGlobalPose and joint_configuration
    getAttributeList(valuesGlobalPose, attributesGlobalPose, xeGlobalPose);
    getAttributeList(valuesJointConfigurationFloat, attributesJointConfiguration, xeJointConfiguration);
// std::cout << "[Debug] 67: Names of Elements" << xeGlobalPose->Name() << xeJointConfiguration->Name() << std::endl;
    Eigen::Map<Eigen::MatrixXf> globalPoseEuler(valuesGlobalPose.data(), 6, 1);
// std::cout << "[Debug] 69 Sizes" << valuesGlobalPose.size() << " " << valuesJointConfigurationFloat.size() << std::flush;
// for (int i = 0; i < valuesGlobalPose.size(); i ++) std::cout << "[Debug] 70 " << valuesGlobalPose.at(i) << std::flush;
// for (int i = 0; i < valuesJointConfigurationFloat.size(); i ++) std::cout << "[Debug] 70 " << valuesJointConfigurationFloat.at(i) << std::flush;	
    Eigen::Map<Eigen::MatrixXf> jointPositions(valuesJointConfigurationFloat.data(), 7, 1);
    jointPositions_=jointPositions;
    poseEulerToPoseMat(globalPose_, globalPoseEuler);
// std::cout << "[Debug] 71" << std::flush;
    //get attributes Symmetries
    
    //Outer Loop
    tinyxml2::XMLElement * xeSymmetry = xeManip->FirstChildElement("symmetry");
    tinyxml2::XMLElement * xeConstraint = NULL;
    tinyxml2::XMLElement * xeJointConfigurationConstraint = NULL;
    
    GripperSymmetry actualGripperSymmetry;
    Constraint actualConstraint;
    SymmetryType actualSymmetryType;
    ConstraintType actualConstraintType;
    
    //Eigen::VectorXi actualJointConfigurationInt;
    //Eigen::VectorXf actualJointConfigurationFloat;   
    std::vector<Eigen::VectorXi> actualJointConfigurationsInt;
    std::vector<Eigen::VectorXi> actualJointConfigurationsFloat;
    //template<typename T>
    //std::vector<T> actualJointConfigurations;
int nrOutLoop=0;
    
    for (;xeSymmetry; xeSymmetry = xeSymmetry->NextSiblingElement())
    {
      
      xeConstraint = xeSymmetry->FirstChildElement("constraint");
      
      
      //get Symmetry Type
      getAttributeList(valuesSymmetry, attributesSymmetry, xeSymmetry);
      getSymmetryTypeFromString(actualSymmetryType, valuesSymmetry.at(0));
      actualGripperSymmetry.symmetryType = actualSymmetryType; //Assigning Gripper Symmetry
      
      //get Constraint Type
      getAttributeList(valuesConstraint, attributesConstraint, xeConstraint);
      getConstraintTypeFromString(actualConstraintType, valuesConstraint.at(0));
      actualConstraint.constraintType = actualConstraintType;
      
      //Middle Loop Constraints
      for(;xeConstraint; xeConstraint = xeConstraint->NextSiblingElement("constraint")) {
	
	xeJointConfigurationConstraint = xeConstraint->FirstChildElement("joint_configuration");
	//InnerLoop Joint Configurations
	for (;xeJointConfigurationConstraint; xeJointConfigurationConstraint = xeJointConfigurationConstraint -> NextSiblingElement("joint_configuration")) 
	{
	  if (actualConstraint.constraintType == BOOL ){	    
	    getAttributeList(valuesJointConfigurationInt, attributesJointConfiguration, xeJointConfigurationConstraint);
	    Eigen::Map<Eigen::MatrixXi> actualJointConfigurationInt(valuesJointConfigurationInt.data(), 7, 1);
	    actualJointConfigurationsInt.push_back(actualJointConfigurationInt);
	  }
	  else if (actualConstraint.constraintType == CONTINUOUS){
	    getAttributeList(valuesJointConfigurationFloat, attributesJointConfiguration, xeJointConfigurationConstraint);
	    Eigen::Map<Eigen::MatrixXi> actualJointConfigurationFloat(valuesJointConfigurationInt.data(), 7, 1);
	    actualJointConfigurationsFloat.push_back(actualJointConfigurationFloat);
	  }
	  else { 
	    std::cout << "actual Constraint Type not correctly defined - exit" << std::endl;
	    exit(EXIT_FAILURE);
	  }
	}	
	//Assigning Constraint and deleting the vectors
	if (actualConstraint.constraintType == BOOL ) actualConstraint.jointConfigurations = actualJointConfigurationsInt;
	else if (actualConstraint.constraintType == CONTINUOUS ) actualConstraint.jointConfigurations = actualJointConfigurationsFloat; //Assignment of Constraint
	actualJointConfigurationsFloat.resize(0);
	actualJointConfigurationsInt.resize(0);	
      }

      //Assigning actual constraint and deleting actualConstraint
      actualGripperSymmetry.symmmetryData.push_back(actualConstraint);
    }
    
    //Assigning actual GripperSymmetry //TODO put into loop
    gripperSymmetry_.push_back( actualGripperSymmetry );
nrOutLoop++;
std::cout << "[Debug] Nr outer loop " << nrOutLoop <<  std::endl;
  }
  else {
  
    std::cout << "Document Parsing Error: " << filePath << "exiting" << std::endl;
  }
}

 void Gripper::printGripperInfo(){
   
   std::cout << "\nGripper Info: \n"; 
   std::cout << "Gripper Identifier: " << gripperIdentifier_ << std::endl;
   std::cout << "Gripper dof: " << dof_ << std::endl;
   std::cout << "Gripper Global Pose:\n" << globalPose_ << std::endl;
   std::cout << "Gripper Joint Configuration:\n" << jointPositions_.transpose() << std::endl;
   std::cout << "Gripper Symmetry States: " << gripperSymmetry_.size()<< std::endl;
   for (int i = 0; i < gripperSymmetry_.size(); i ++){
     std::cout << "Gripper Symmetry at " << i << std::endl;
     gripperSymmetry_.at(i).print();
   }
 }






















