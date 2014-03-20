#include "GraspSymmetrizer/symmetry.h"


void reflectionMatrix(Eigen::Matrix4f & reflectionMat, const Layer3D layer){
  
  Point3D n,trans;
  
  n = ((layer.point2-layer.point1).cross(layer.point3-layer.point1)).normalized(); //normalvector
  trans = 2*n*fabs((n.transpose()*layer.point1).value());
  
  reflectionMat.col(0) << 1-2*n(0)*n(0), -2*n(1)*n(0), -2*n(2)*n(0),0;
  reflectionMat.col(1) <<  -2*n(0)*n(1),1-2*n(1)*n(1), -2*n(2)*n(1),0;
  reflectionMat.col(2) <<  -2*n(0)*n(2), -2*n(1)*n(2),1-2*n(2)*n(2),0;
  reflectionMat.col(3) << trans,1;
  
}

void axisRotationMatrix(Eigen::Matrix4f & axisMat, const Axis3D axis, const float alpha){

  Point3D w,n2,ex; //tangentialvector
	
	w=(axis.point2-axis.point1).normalized(); //is e_z
	
	//create normalvector for w
	Point3D viewUp;
	viewUp << 0,1,0;
	
// std::cout << "[Debug:] 28" << fabs(viewUp.transpose()*w) - 1.0f << std::endl;


	if ( (fabs(viewUp.cross(w).norm()) < 0.001f)) //if they are parallel, thats bad
	{
// std::cout << "[Debug:] 32, in if condition" << std::endl;
	  viewUp << 0,0,1;
	}
	
	n2=w.cross(viewUp).normalized();
	
	ex=n2.cross(w);
	
	Eigen::Matrix4f axislocal; // transform from axis local system to object frame
	axislocal.col(0) << ex,0;
	axislocal.col(1) << n2,0;
	axislocal.col(2) << w, 0;
	axislocal.col(3) << axis.point1,1;
	
	Eigen::Matrix3f rotZ;
	rotZ = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ());
	
	Eigen::Matrix4f rot4d;
	rot4d.col(0) << rotZ.col(0),0;
	rot4d.col(1) << rotZ.col(1),0;
	rot4d.col(2) << rotZ.col(2),0;
	rot4d.col(3) << 0,0,0,1;
	
	Eigen::Matrix4f axialtrans; //-> local KO -> rotation by alpha -> object frame
		
	axialtrans = axislocal*rot4d*axislocal.inverse();
	axisMat = axialtrans;
}

void axisRotationMatrixesN(std::vector<Eigen::Matrix4f> & axisMatrixes, const Axis3D axis, const float alpha, const int nrRotations){
 
  //pushing back!
  Eigen::Matrix4f actualRotMat;
  
  if (axisMatrixes.size()==0)
  {
    for (int i = 0; i < nrRotations; i++)
    {
      axisRotationMatrix(actualRotMat, axis, alpha*(i+1) );
      axisMatrixes.push_back(actualRotMat);
    }
  }
  else
  {
    std::cout << "expects input vector with size 0!" << std::endl;
  }
  
}//subsequent single rotation, gives back all but identity rotation

// bool SymmetryOperation::checkConstraintBool(const std::vector<Eigen::VectorXi> & jointConstraints, const std::vector<Eigen::VectorXf> & jointConfigurations ){
//  
//   
//   
// }




bool SymmetryOperation::checkConstraintContinuous(const std::vector<Eigen::VectorXf> & jointConstraints, const std::vector<Eigen::VectorXf> & jointConfigurations ){
 
  float sim_threshold= 3*M_PI/180.0; //3 deg
  bool constraintOk = true;
  float nan  = std::numeric_limits<float>::quiet_NaN();
  //
  for (int i = 0; i < jointConstraints.size(); i++)
  {
    
    for (int j = 0; j < jointConfigurations.size(); j++)
    {
      
      for (int k = 0; k < jointConstraints.at(i).size(); k ++)
      {
	if ( constraintOk && jointConstraints.at(i)(k) == nan) continue;
	else if (constraintOk && fabs(jointConstraints.at(i)(k) - jointConfigurations.at(j)(k) ) < sim_threshold  ) continue;
	else constraintOk = false;
      }
      
    }
    
  }
  return constraintOk;  
}

void SymmetryOperation::flipJointConstraintBool(const std::vector<Eigen::VectorXi> & jointConstraints, std::vector<Eigen::VectorXf> & jointConfigurations ){
 
  float valueTemp;
  int indexTemp=-1;
  //Flip Bool Bit must be disjoint set of 2 bits set, I do not check this here
// std::cout << "[DEBUG] 121: printing joint_constraints" << std::endl;

  for (int i = 0; i < jointConstraints.size(); i++)
  {
//     std::cout << "[DEBUG] 125: printing joint_constraints at i " << i << jointConstraints.at(i).transpose() << std::endl;
    
    for (int j = 0; j < jointConfigurations.size(); j++)
    {
//       std::cout << "[DEBUG] 125: printing joint_config before inner loop at j " << j << jointConfigurations.at(j).transpose() << std::endl;
      for (int k = 0; k < jointConstraints.at(i).size(); k ++)
      {
	if( jointConstraints.at(i)(k) == 1 && indexTemp ==-1)
	{
	  valueTemp = jointConfigurations.at(j)(k);
	  indexTemp = k;
	}
	else if (jointConstraints.at(i)(k) == 1 && indexTemp != -1)
	{
	  jointConfigurations.at(j)(indexTemp) = jointConfigurations.at(j)(k);
	  jointConfigurations.at(j)(k) = valueTemp;	  
	}
      }
      indexTemp=-1;
//       std::cout << "[DEBUG] 125: printing joint_config after inner loop at j " << j << jointConfigurations.at(j).transpose() << std::endl;
      
    }
    
  }
  
}

void SymmetryOperation::reflectGrasps(const Layer3D & objectLayer, const Layer3D & gripperLayer, std::vector<Grasp> & grasp_list, const std::vector<Eigen::VectorXi> & jointConstraints){
 
  std::vector<Grasp> generated_grasps;

  //Get reflection Matrices  
  Eigen::Matrix4f graspitFlip;
  graspitFlip << 2,0,0,0,
		 0,0,1,0,
		 0,1,0,0,
		 0,0,0,1;
  Eigen::Matrix4f objectReflexMatrix;
  Eigen::Matrix4f gripperReflexMatrix;
  //Transform gripper layer also with grasp layer! only invert signs, hmm point by point
  reflectionMatrix(objectReflexMatrix, objectLayer);
  reflectionMatrix(gripperReflexMatrix, gripperLayer);
  
std::cout << "[DEBUG] 163 - Gripper Reflection Matrix" << gripperReflexMatrix << std::endl;
    
  //Apply rotation matrices to all hand_poses!
  
  for(int i = 0; i < grasp_list.size(); i++)
  {
    //Insert original grasps on first position of generated:
    generated_grasps.push_back(grasp_list.at(i));
   
          //generate new grasp with copy-constructor
      Grasp new_grasp(grasp_list.at(i));
      for(int k = 0; k < new_grasp.handPoses_.size(); k++)
      {

	new_grasp.handPoses_.at(k) = objectReflexMatrix*new_grasp.handPoses_.at(k)*gripperReflexMatrix;

	//TransformLinkAtoB(new_grasp.handPoses_.at(k) ,graspitFlip.inverse() ); //to be tested
	flipJointConstraintBool(jointConstraints, new_grasp.jointPositions_);
      }
      
      generated_grasps.push_back(new_grasp);
      
  } 
 grasp_list = generated_grasps;  
  
  
}
  
 

void SymmetryOperation::rotateGrasps(const Axis3D axis, std::vector<Grasp> & grasp_list){
 
  std::vector<Grasp> generated_grasps;

  //Get rotation Matrices  
  std::vector<Eigen::Matrix4f> axisMatrixes;
  float rotAngle = 2*M_PI/(rotationSamplingNr_+1);
  
  axisRotationMatrixesN(axisMatrixes, axis, rotAngle, rotationSamplingNr_);
  
  //Apply rotation matrices to all hand_poses!
  
  for(int i = 0; i < grasp_list.size(); i++)
  {
    //Insert original grasps on first position of generated:
    generated_grasps.push_back(grasp_list.at(i));
   
    for(int j = 0; j < axisMatrixes.size(); j++)
    {
      //generate new grasp with copy-constructor
      Grasp new_grasp(grasp_list.at(i));
      for(int k = 0; k < new_grasp.handPoses_.size(); k++)
      {
	new_grasp.handPoses_.at(k) = axisMatrixes.at(j)*new_grasp.handPoses_.at(k);
      }
      
      generated_grasps.push_back(new_grasp);
    }    
  } 
 grasp_list = generated_grasps;  
}


void SymmetryOperation::getActiveGripperSymmetry(const Grasp & grasp, SymmetryType symType){
 
  //check for c3
  //blub
  
  
}




void SymmetryOperation::computeSymmetries(){
  
  //Cases depending on Object Symmetry
  resultingGrasps_.push_back(inputGrasp_);
  
  switch (objectSymmetry_.symmetryType)
  {
  case SINGLEPLANE: 
    {
      
      Layer3D objectPlane = (boost::get<SinglePlane>(objectSymmetry_.symmetryData)).plane1;
      Layer3D gripperPlane = (boost::get<SinglePlane>(gripperSymmetry_.symmetryData)).plane1;
      std::vector<Eigen::VectorXi> jointCsBool = boost::get< std::vector<Eigen::VectorXi> > ( gripperSymmetry_.symmetryConstraint.at(0).jointConfigurations);
        
      reflectGrasps( objectPlane, gripperPlane, resultingGrasps_, jointCsBool );
    }
    break;
  case DOUBLEPLANE: 
    {
      Layer3D objectPlane1 = (boost::get<DoublePlane>(objectSymmetry_.symmetryData)).plane1;
      Layer3D objectPlane2 = (boost::get<DoublePlane>(objectSymmetry_.symmetryData)).plane2;
      Layer3D gripperPlane = (boost::get<SinglePlane>(gripperSymmetry_.symmetryData)).plane1;
      std::vector<Eigen::VectorXi> jointCsBool = boost::get< std::vector<Eigen::VectorXi> > ( gripperSymmetry_.symmetryConstraint.at(0).jointConfigurations);
        
      reflectGrasps( objectPlane1, gripperPlane, resultingGrasps_, jointCsBool );
      reflectGrasps( objectPlane2, gripperPlane, resultingGrasps_, jointCsBool );

    }
    break;
  case TRIPLEPLANE: 
    {
      Layer3D objectPlane1 = (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane1;
      Layer3D objectPlane2 = (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane2;
      Layer3D objectPlane3 = (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane3;
      Layer3D gripperPlane = (boost::get<SinglePlane>(gripperSymmetry_.symmetryData)).plane1;
      std::vector<Eigen::VectorXi> jointCsBool = boost::get< std::vector<Eigen::VectorXi> > ( gripperSymmetry_.symmetryConstraint.at(0).jointConfigurations);
        
      reflectGrasps( objectPlane1, gripperPlane, resultingGrasps_, jointCsBool );
      reflectGrasps( objectPlane2, gripperPlane, resultingGrasps_, jointCsBool );
      reflectGrasps( objectPlane3, gripperPlane, resultingGrasps_, jointCsBool );
      
    }
    break;
  case AXIAL: 
    {
      Axis3D objectAxis = (boost::get<Axial>(objectSymmetry_.symmetryData)).axis1;
      rotateGrasps( objectAxis, resultingGrasps_ );
    }
    break;
  case AXIALSINGLEPLANE: 
    {
       
       Axis3D objectAxis = (boost::get<AxialSinglePlane>(objectSymmetry_.symmetryData)).axis1;
       rotateGrasps( objectAxis, resultingGrasps_);
       
       Layer3D objectPlane = (boost::get<AxialSinglePlane>(objectSymmetry_.symmetryData)).plane1;
       Layer3D gripperPlane = (boost::get<SinglePlane>(gripperSymmetry_.symmetryData)).plane1;
       std::vector<Eigen::VectorXi> jointCsBool = boost::get< std::vector<Eigen::VectorXi> > ( gripperSymmetry_.symmetryConstraint.at(0).jointConfigurations);
      
       reflectGrasps( objectPlane, gripperPlane, resultingGrasps_, jointCsBool );

    }
    break;
  default: 
    std::cout << "Objectsymmetry not correctly defined, exiting!\n";
    exit(EXIT_FAILURE);
    break;
  }
  
  std::cout << "Symmetry Successfully computed!\n";

}
