#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/util.h"

inline bool file_exists (const std::string& name) {
    std::ifstream f(name.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }   
}


void Grasp::setGripper(const Gripper & gripper){
  gripperIdentifier_ = gripper.getName();
  dof_=gripper.getDof();
}

void Grasp::loadFromXml(tinyxml2::XMLElement* graspElement){
 
  std::vector<std::string> attributesGrasp,  
			   attributesJointConfiguration, 
			   attributesPoseConfiguration,
			   valuesGrasp;
  std::vector<float> valuesJointConfiguration,
		     valuesPoseConfiguration;
  attributesGrasp.push_back("object_identifier");
  attributesGrasp.push_back("gripper_identifier");
  
  attributesPoseConfiguration.push_back("x");
  attributesPoseConfiguration.push_back("y");
  attributesPoseConfiguration.push_back("z");
  attributesPoseConfiguration.push_back("roll");
  attributesPoseConfiguration.push_back("pitch");
  attributesPoseConfiguration.push_back("yaw");
  
  //here Schunk hand attributes Hardcoded - TODO replace generic
  for(int i = 1; i < 8; i++){
    std::string jointName = "j" + intToString(i);
    attributesJointConfiguration.push_back(jointName);
  }
  
  //check pointer
  if (graspElement) {
    
    //get Grasp attributes
    getAttributeList(valuesGrasp, attributesGrasp, graspElement);
    
    objectIdentifier_=valuesGrasp.at(0);
    gripperIdentifier_=valuesGrasp.at(1);
    valuesGrasp.resize(0);

    //get hand_joint_sequences    
    tinyxml2::XMLElement *handJointSeqPtr = graspElement->FirstChildElement("hand_joint_sequence");
    tinyxml2::XMLElement *jointConfigPtr = handJointSeqPtr->FirstChildElement("joint_configuration");
    
    for(;jointConfigPtr; jointConfigPtr=jointConfigPtr->NextSiblingElement("joint_configuration"))
    {
      getAttributeList(valuesJointConfiguration, attributesJointConfiguration, jointConfigPtr);
      Eigen::Map<Eigen::MatrixXf> actualJointConfig(valuesJointConfiguration.data(), 7, 1);
      valuesJointConfiguration.resize(0);
      
      jointPositions_.push_back(actualJointConfig); //assignment
    }
    
    //get hand_pose_sequence
    tinyxml2::XMLElement *handPoseSeqPtr = graspElement->FirstChildElement("hand_pose_sequence");
    tinyxml2::XMLElement *poseConfigPtr = handPoseSeqPtr->FirstChildElement("pose_configuration");
    PoseMat actualPoseMat;
    
    for(;poseConfigPtr; poseConfigPtr=poseConfigPtr->NextSiblingElement("pose_configuration"))
    {
// std::cout << "[Debug:] 64 - In Pose Loop" << std::endl;
      getAttributeList(valuesPoseConfiguration, attributesPoseConfiguration, poseConfigPtr);
      Eigen::Map<Eigen::MatrixXf> actualPoseConfig(valuesPoseConfiguration.data(), 6, 1);
      poseEulerToPoseMat( actualPoseMat, actualPoseConfig);
      valuesPoseConfiguration.resize(0);
      
      handPoses_.push_back(actualPoseMat); //assignment
    }
  }
  else 
  {
    std::cout << "[Error] grasp XMLPtr is NULL, exiting\n";
    exit(EXIT_FAILURE);
  }
}

void Grasp::printGrasp() const {
  
  std::cout << "Gripper: " << gripperIdentifier_ << std::endl;
  std::cout << "Object: " << objectIdentifier_ << std::endl;
  std::cout << "dof: " << dof_ << std::endl;
  std::cout << "HandposeSequence: Nr: " << handPoses_.size() << std::endl;
  
  for (int i = 0; i < handPoses_.size(); i++) 
    std::cout << handPoses_.at(i) << "\n" << std::endl;
  
  std::cout << "HandJointSequence: Nr: " << jointPositions_.size() << std::endl;
  for (int i = 0; i < jointPositions_.size(); i++) 
    std::cout << jointPositions_.at(i).transpose() << std::endl;
  
}

void Grasp::loadFromGraspItXml(const std::string filePath){

  tinyxml2::XMLDocument doc;
  doc.LoadFile(filePath.c_str());
  
  if (!doc.Error()) {
    
    //GraspMatrixes
    PoseMat objectPose = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf orientationObject;
    Eigen::Vector3f positionObject;
    
    PoseMat handPose  = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf orientationHand;
    Eigen::Vector3f positionHand;
    
    Eigen::VectorXf actPose(7);
    Eigen::VectorXf actJoints(8);
    Eigen::VectorXf myConventionJoints(7);
    //Pointers to XML Elements
    tinyxml2::XMLElement* xeParent = doc.FirstChildElement("world");
    tinyxml2::XMLElement* xeObjectTransform = xeParent->FirstChildElement("graspableBody")->FirstChildElement("transform")->FirstChildElement("fullTransform");
    tinyxml2::XMLElement* xeHandTransform = xeParent->FirstChildElement("robot")->FirstChildElement("transform")->FirstChildElement("fullTransform");
    tinyxml2::XMLElement* xeFingerJoints = xeParent->FirstChildElement("robot")->FirstChildElement("dofValues");
  
    const char* poseObjectChar = xeObjectTransform->GetText();
    const char* poseHandChar = xeHandTransform->GetText();
    const char* jointsFingerChar = xeFingerJoints->GetText();
    
    std::string poseObject(poseObjectChar);
    std::string poseHand(poseHandChar);
    std::string jointsFinger(jointsFingerChar);
    
    //Removing brackets ( and [ from the string
    std::vector<int> replacePosObj;
    replacePosObj.push_back(poseObject.find("("));
    replacePosObj.push_back(poseObject.find(")"));
    replacePosObj.push_back(poseObject.find("["));
    replacePosObj.push_back(poseObject.find("]"));
    
    std::vector<int> replacePosHand;
    replacePosHand.push_back(poseHand.find("("));
    replacePosHand.push_back(poseHand.find(")"));
    replacePosHand.push_back(poseHand.find("["));
    replacePosHand.push_back(poseHand.find("]"));
    
    //same for the other transform, so just use same positions
      for(int i = 0; i < replacePosObj.size(); i++){
	poseObject.replace(replacePosObj.at(i), 1, " ");
      }
      
      for(int i = 0; i < replacePosHand.size(); i++){
	poseHand.replace(replacePosHand.at(i), 1, " ");
      }
  
    
// std::cout << "[Debug:] Hand and Object Pose from GraspIt as a string" << poseObject << poseHand << std::endl;
    
    actPose = StringToMat( poseObject, 1 , 7 ).transpose();
    positionObject = actPose.tail(3)*0.001; //conversion to m
    orientationObject.w() = actPose(0);
    orientationObject.x() = actPose(1);
    orientationObject.y() = actPose(2);
    orientationObject.z() = actPose(3);
    
    actPose = StringToMat( poseHand, 1 , 7 ).transpose();
    
    positionHand = actPose.tail(3)*0.001; //conversion to m
    orientationHand.w() = actPose(0);
    orientationHand.x() = actPose(1);
    orientationHand.y() = actPose(2);
    orientationHand.z() = actPose(3);

    objectPose.block<3,3>(0,0) = orientationObject.toRotationMatrix();
    objectPose.col(3) << positionObject,1;
    
// std::cout << "[Debug:] ObjectPose \n" << objectPose << std::endl;
    
    handPose.block<3,3>(0,0) = orientationHand.toRotationMatrix();
    handPose.col(3) << positionHand,1;
// std::cout << "[Debug:] HandPose\n" << handPose << std::endl;
  //      cout <<"[debug] 4 Matrixes from Quaternion ..." << positionHand << positionObject << orientationHand.toRotationMatrix() << orientationObject.toRotationMatrix() << endl;
    
    PoseMat graspPose = objectPose.inverse()*handPose;
    
    //One should check here for a global-Pose Offset, by comparing the kinematics file
    
// std::cout << "[Debug:] GraspPose: \n" << graspPose << std::endl;
    
  
  //Getting hand joints:
  actJoints =  StringToMat( jointsFinger, 1 , 8 ).transpose();
  
  //Add joint reordering:
  myConventionJoints(0) = actJoints(0);
  myConventionJoints(1) = actJoints(6);
  myConventionJoints(2) = actJoints(7);
  myConventionJoints(3) = actJoints(1);
  myConventionJoints(4) = actJoints(2);
  myConventionJoints(5) = actJoints(3);
  myConventionJoints(6) = actJoints(4);

// std::cout << "[Debug:] HandJoints: \n" << myConventionJoints << std::endl;
  
  handPoses_.push_back(graspPose);
  jointPositions_.push_back(myConventionJoints);

  
    
  }
  else {  
    std::cout << "[Error] Document Parsing Error: " << filePath << "\n exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void Grasp::addToXml(std::string filePath){
  
  //write to .xml directly  
  tinyxml2::XMLDocument doc;
  
   if (file_exists(filePath)){
   
    doc.LoadFile(filePath.c_str());
    
    if (!doc.Error()) {
      std::cout << "Document successfully loaded: " << filePath << std::endl;
    }
    else {
      std::cout << "[Error] Document Parsing Error: " << filePath << "not overwriting - exit!" << std::endl;
      exit(EXIT_FAILURE);
    }
   }
   else {
  
  //Add body
  static const char* xml =
		"<?xml version=\"1.0\"?>"
		"<graspdatabase>"
		"</graspdatabase>";
  
  doc.Parse( xml );
   }
  
  tinyxml2::XMLElement* graspDbPtr = doc.FirstChildElement("graspdatabase");
  
  std::vector<std::string> elementNames;
  
  elementNames.push_back("grasp");
  elementNames.push_back("hand_joint_sequence");
  elementNames.push_back("hand_pose_sequence");
  elementNames.push_back("joint_configuration");
  elementNames.push_back("pose_configuration");
  
  //Create and Insert XML Elements 
  
  tinyxml2::XMLElement* graspPtrList[5];
  std::vector<tinyxml2::XMLElement*> jointSeqPtrList;
  std::vector<tinyxml2::XMLElement*> poseSeqPtrList;
  
  for(int i = 0; i < 5; i++){  
    graspPtrList[i] = doc.NewElement(elementNames.at(i).c_str());
  }
  
   for (int i = 0; i < jointPositions_.size(); i ++){
    jointSeqPtrList.push_back(doc.NewElement(elementNames.at(3).c_str()));
    graspPtrList[1]->InsertEndChild(jointSeqPtrList.at(i));
   }
   
   for (int i = 0; i < handPoses_.size(); i ++){
    poseSeqPtrList.push_back(doc.NewElement(elementNames.at(4).c_str()));
   graspPtrList[2]->InsertEndChild(poseSeqPtrList.at(i));
   }
  
  graspDbPtr->InsertEndChild(graspPtrList[0]); 
  graspPtrList[0]->InsertEndChild(graspPtrList[1]);
  graspPtrList[0]->InsertEndChild(graspPtrList[2]);
  
  
  

  std::vector<std::string> attributesGrasp,  
			    attributesJointConfiguration, 
			    attributesPoseConfiguration,
			    valuesGrasp;
			    
  std::vector< std::vector<std::string> >  valuesJointConfiguration,	     
			    valuesPoseConfiguration;
			   
  attributesGrasp.push_back("object_identifier");
  attributesGrasp.push_back("gripper_identifier");
  
  attributesPoseConfiguration.push_back("x");
  attributesPoseConfiguration.push_back("y");
  attributesPoseConfiguration.push_back("z");
  attributesPoseConfiguration.push_back("roll");
  attributesPoseConfiguration.push_back("pitch");
  attributesPoseConfiguration.push_back("yaw");
  
  //here Schunk hand attributes Hardcoded - TODO replace generic
  for(int i = 1; i < 8; i++){
    std::string jointName = "j" + intToString(i);
    attributesJointConfiguration.push_back(jointName);
  }
  
  //Convert internal values to std::vector<string>
  valuesGrasp.push_back(objectIdentifier_);
  valuesGrasp.push_back(gripperIdentifier_);
  
  PoseEuler actualEulerPose;
  for(int i=0; i < jointPositions_.size(); i++)
  {
  valuesJointConfiguration.push_back( VecToStdVecString(jointPositions_.at(i)) );
  }
  for(int i=0; i < handPoses_.size(); i++)
  {
    poseMatToPoseEuler(actualEulerPose,handPoses_.at(i));
    valuesPoseConfiguration.push_back( VecToStdVecString(actualEulerPose) );
  }
    
    //check pointer
    tinyxml2::XMLElement* graspElement = graspPtrList[0];
  if (graspElement) {
    
    //set Grasp attributes
    setAttributeList(valuesGrasp, attributesGrasp, graspElement);

    //get hand_joint_sequences    
    tinyxml2::XMLElement *handJointSeqPtr = graspElement->FirstChildElement("hand_joint_sequence");
    tinyxml2::XMLElement *jointConfigPtr = handJointSeqPtr->FirstChildElement("joint_configuration");
    
    int iterJoint = 0;
    for(;jointConfigPtr; jointConfigPtr=jointConfigPtr->NextSiblingElement("joint_configuration"))
    {
      setAttributeList(valuesJointConfiguration.at(iterJoint), attributesJointConfiguration, jointConfigPtr);
      iterJoint++;
    }
    
    //get hand_pose_sequence
    tinyxml2::XMLElement *handPoseSeqPtr = graspElement->FirstChildElement("hand_pose_sequence");
    tinyxml2::XMLElement *poseConfigPtr = handPoseSeqPtr->FirstChildElement("pose_configuration");

    int iterPose = 0;
    for(;poseConfigPtr; poseConfigPtr=poseConfigPtr->NextSiblingElement("pose_configuration"))
    {
// std::cout << "[Debug:] 64 - In Pose Loop" << std::endl;
      setAttributeList(valuesPoseConfiguration.at(iterPose), attributesPoseConfiguration, poseConfigPtr);
      iterPose++;
    }
    
    doc.SaveFile(filePath.c_str());
  }
  else 
  {
    std::cout << "[Error] grasp XMLPtr is NULL, exiting\n";
    exit(EXIT_FAILURE);
  }

}
   
void GraspDatabase::loadFromXml(const std::string filePath){
 
  tinyxml2::XMLDocument doc;
  doc.LoadFile(filePath.c_str());
  
  if (!doc.Error()) {
    std::cout << "Document successfully loaded: " << filePath << std::endl;
  }
  else {
    std::cout << "[Error] Document Parsing Error: " << filePath << " exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  tinyxml2::XMLElement * graspPtr = doc.FirstChildElement()->FirstChildElement("grasp");
  
  if (graspPtr) {
    for(;graspPtr ; graspPtr=graspPtr->NextSiblingElement("grasp"))
    {
      Grasp grasp(graspPtr);
      graspDb_.push_back(grasp);
    }
  }
  else 
  {
    std::cout << "[Error] grasp XMLPtr is NULL, exiting\n";
    exit(EXIT_FAILURE);
  }
  
}


//Should be a layout file with already the correct robot and object filepaths!
void Grasp::saveToGraspItXml(const std::string layoutFilePath, const std::string saveFilePath){
 
  //Should be a layout file with already the correct robot and object filepaths!
  tinyxml2::XMLDocument doc;
  doc.LoadFile(layoutFilePath.c_str());
  
  if (!doc.Error()) {
    
    //GraspMatrixes
    PoseMat objectPose = Eigen::Matrix4f::Identity();
    PoseQuat objectPoseQuat;
    PoseQuat gripperPoseQuat;
    Eigen::VectorXf actJoints(8);
    Eigen::VectorXf myConventionJoints = jointPositions_.at(0); //Takes the 0th entry of the grasp
    
    std::string objectPoseStr;
    std::string gripperPoseStr;
    std::string handJointStr;
    
    poseMatToPoseQuat(objectPoseQuat, objectPose);
    poseMatToPoseQuat(gripperPoseQuat, handPoses_.at(0));
    
    //Generate the strings:
    
    std::stringstream ss1, ss2;
    
    ss1 << "(" << gripperPoseQuat(3) << " " << 
                  gripperPoseQuat(4) << " " << gripperPoseQuat(5) << " " 
		  << gripperPoseQuat(6) << ")[";
    ss1 << gripperPoseQuat(0)*1000 << " " << 
                  gripperPoseQuat(1)*1000 << " " << gripperPoseQuat(2)*1000 << "]";
		  
    ss2 << "(" << objectPoseQuat(3) << " " << 
                  objectPoseQuat(4) << " " << objectPoseQuat(5) << " " 
		  << objectPoseQuat(6) << ")[";
    ss2 << objectPoseQuat(0)*1000 << " " << 
                  objectPoseQuat(1)*1000 << " " << objectPoseQuat(2)*1000 << "]";

	gripperPoseStr = ss1.str();
	objectPoseStr = ss2.str();
    //Conversion to graspIt joint convention:
    //Add joint reordering:
    
    actJoints(0) = myConventionJoints(0);
    actJoints(1) = myConventionJoints(3);
    actJoints(2) = myConventionJoints(4);
    actJoints(3) = myConventionJoints(5);
    actJoints(4) = myConventionJoints(6);
    actJoints(5) = 0;
    actJoints(6) = myConventionJoints(1);
    actJoints(7) = myConventionJoints(2);
    
    handJointStr = MatToString(actJoints);
    
    //Pointers to XML Elements
    tinyxml2::XMLElement* xeParent = doc.FirstChildElement("world");
    tinyxml2::XMLElement* xeObjectTransform = xeParent->FirstChildElement("graspableBody")->FirstChildElement("transform")->FirstChildElement("fullTransform");
    tinyxml2::XMLElement* xeHandTransform = xeParent->FirstChildElement("robot")->FirstChildElement("transform")->FirstChildElement("fullTransform");
    tinyxml2::XMLElement* xeFingerJoints = xeParent->FirstChildElement("robot")->FirstChildElement("dofValues");
  
    xeObjectTransform->SetText(objectPoseStr.c_str());
    xeHandTransform->SetText(gripperPoseStr.c_str());
    xeFingerJoints->SetText(handJointStr.c_str());
    
    doc.SaveFile(saveFilePath.c_str());
    
  }
  else {  
    std::cout << "[Error] Document Parsing Error: " << layoutFilePath << "exiting" << std::endl;
  }
  
  
}

void GraspDatabase::printGraspDatabase(){
 
  std::cout << "Grasp Database: " << std::endl;
  for(int i = 0; i < graspDb_.size(); i++)
  {
    std::cout << "Grasp Number: " << i << std::endl;
    graspDb_.at(i).printGrasp();
  }
  
}

void GraspDatabase::saveToXml(const std::string filePath){
  
  for (int i = 0; i < graspDb_.size(); i++){   
    graspDb_.at(i).addToXml(filePath);    
  }
}




//TODO: maybe add extrapolator function for a single grasp Sequence along approach direction


