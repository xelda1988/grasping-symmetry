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

	if ( (fabs(viewUp.transpose()*w) - 1.0f < 0.001f)) //if they are parallel, thats bad
	{
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






/*
void SymmetryOperation::computeSymmetries(){
  
  //Cases depending on Object Symmetry
  resultingGrasps_.push_back(inputGrasp_);
  
  switch (objectSymmetry_.symmetryType)
  {
  case SINGLEPLANE: 
    {
      reflectGrasps( (boost::get<SinglePlane>(objectSymmetry_.symmetryData)).plane1 ); //does Operation on resultingGrasps
    }
    break;
  case DOUBLEPLANE: 
    {
      reflectGrasps( (boost::get<DoublePlane>(objectSymmetry_.symmetryData)).plane1 );
      reflectGrasps( (boost::get<DoublePlane>(objectSymmetry_.symmetryData)).plane2 );
    }
    break;
  case TRIPLEPLANE: 
    {
      reflectGrasps( (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane1);
      reflectGrasps( (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane2);
      reflectGrasps( (boost::get<TriplePlane>(objectSymmetry_.symmetryData)).plane3);      
    }
    break;
  case AXIAL: 
    {
      rotateGrasps( (boost::get<Axial>(objectSymmetry_.symmetryData)).axis1);
    }
    break;
  case AXIALSINGLEPLANE: 
    {
      rotateGrasps( (boost::get<AxialSinglePlane>(objectSymmetry_.symmetryData)).axis1);
      reflectGrasps( (boost::get<AxialSinglePlane>(objectSymmetry_.symmetryData)).plane1);
    }
    break;
  default: 
    std::cout << "Objectsymmetry not correctly defined, exiting!\n";
    exit(EXIT_FAILURE);
    break;
  }
  
  std::cout << "Symmetry Successfully computed!\n";

}
*/