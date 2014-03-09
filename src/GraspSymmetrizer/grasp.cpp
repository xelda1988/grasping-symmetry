#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/util.h"


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
    std::cout << "Error: grasp XMLPtr is NULL, exiting\n";
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
  
    
std::cout << "[Debug:] Hand and Object Pose from GraspIt as a string" << poseObject << poseHand << std::endl;
    
    actPose = StringToMat( poseObject, 1 , 7 ).transpose();
    positionObject = actPose.tail(3)*0.001; //conversion to m
    orientationObject.w() = actPose(0);
    orientationObject.x() = actPose(1);
    orientationObject.y() = actPose(2);
    orientationObject.z() = actPose(3);
    
    actPose = StringToMat( poseHand, 1 , 7 ).transpose();
    
    positionHand = actPose.tail(3)*0.001;
    orientationHand.w() = actPose(0);
    orientationHand.x() = actPose(1);
    orientationHand.y() = actPose(2);
    orientationHand.z() = actPose(3);

    objectPose.block<3,3>(0,0) = orientationObject.toRotationMatrix();
    objectPose.col(3) << positionObject,1;
    
std::cout << "[Debug:] ObjectPose \n" << objectPose << std::endl;
    
    handPose.block<3,3>(0,0) = orientationHand.toRotationMatrix();
    handPose.col(3) << positionHand,1;
std::cout << "[Debug:] HandPose\n" << handPose << std::endl;
  //      cout <<"[debug] 4 Matrixes from Quaternion ..." << positionHand << positionObject << orientationHand.toRotationMatrix() << orientationObject.toRotationMatrix() << endl;
    
    PoseMat graspPose = objectPose.inverse()*handPose;
    
std::cout << "[Debug:] GraspPose: \n" << graspPose << std::endl;
    
    
    
  }
  else {  
    std::cout << "Document Parsing Error: " << filePath << "exiting" << std::endl;
  }
}

void Grasp::addToXml(std::string filePath){
  
  //write to .xml directly  
  tinyxml2::XMLDocument doc;
  doc.LoadFile(filePath.c_str());
  
  if (!doc.Error()) {
    std::cout << "Document successfully loaded: " << filePath << std::endl;
  }
  else {
    std::cout << "Document Parsing Error: " << filePath << "exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  //Add body
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
    std::cout << "Error: grasp XMLPtr is NULL, exiting\n";
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
    std::cout << "Document Parsing Error: " << filePath << " exiting" << std::endl;
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
    std::cout << "Error: grasp XMLPtr is NULL, exiting\n";
    exit(EXIT_FAILURE);
  }
  
}

void GraspDatabase::printGraspDatabase(){
 
  std::cout << "Grasp Database: " << std::endl;
  for(int i = 0; i < graspDb_.size(); i++)
  {
    graspDb_.at(i).printGrasp();
  }
  
}













//TODO: maybe add extrapolator function for a single grasp Sequence along approach direction


