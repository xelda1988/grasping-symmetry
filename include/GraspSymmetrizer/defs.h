#ifndef GRASPSYMMETRIZER_DEFS_H_
#define GRASPSYMMETRIZER_DEFS_H_

#include <string>
#include <Eigen/Dense>
#include <vector>
#include <boost/variant.hpp>

//Simple Types

  
typedef Eigen::Vector3f Point3D;
typedef Eigen::Matrix4f PoseMat;
typedef Eigen::Matrix < float, 6, 1> PoseEuler; // ZYX Convention (x,y,z,r,p,y)
typedef Eigen::Matrix < float, 7, 1> PoseQuat; // Convention (x_pos,y_pos,z_pos,w, x_ori,y_ori,z_ori)

struct Axis3D {
  Point3D point1;
  Point3D point2;
  void print(){
    std::cout << "Axis3D: \n";
    std::cout << point1.transpose() << std::endl;
    std::cout << point2.transpose() << std::endl;
  }
  void scale(float scaleFactor){
    point1=point1*scaleFactor;
    point2=point2*scaleFactor;
  }
};

struct Layer3D {
  Point3D point1;
  Point3D point2;
  Point3D point3;
  void print(){
    std::cout << "Layer3D: \n";
    std::cout << point1.transpose() << std::endl;
    std::cout << point2.transpose() << std::endl;
    std::cout << point3.transpose() << std::endl;
  }
  void scale(float scaleFactor){
    point1=point1*scaleFactor;
    point2=point2*scaleFactor;
    point3=point3*scaleFactor;
  }
};

//Generic Symmetries
enum SymmetryType { 
  SINGLEPLANE = 1,
  DOUBLEPLANE, 
  TRIPLEPLANE,
  AXIAL,
  AXIALSINGLEPLANE,
  C3, //Hand all 120degree rotation
  NONE
};

enum ConstraintType {
  BOOL = 1,
  CONTINUOUS
};

struct Axial {
  Axis3D axis1;
  void print(){
    std::cout << "Axial: \n";
    axis1.print();
  }
  void scale(float scaleFactor){
   axis1.scale(scaleFactor); 
  }
};

struct AxialSinglePlane {
  Axis3D axis1;
  Layer3D plane1;
  void print(){
    std::cout << "AxialSinglePlane: \n";
    axis1.print();
    plane1.print();
  }
  void scale(float scaleFactor){
   axis1.scale(scaleFactor); 
   plane1.scale(scaleFactor);
  }
};

struct SinglePlane {
  Layer3D plane1;
  void print(){
    std::cout << "SinglePlane: \n";
    plane1.print();
  }
  void scale(float scaleFactor){
// std::cout << "[Debug] in scale" << std::endl;
// std::cout << "Plane before scaling" << std::endl;
   plane1.print();
   plane1.scale(scaleFactor);
   plane1.print();
  }
  
};

struct DoublePlane {
  Layer3D plane1;
  Layer3D plane2;
  Layer3D plane3;
  void print(){
    std::cout << "DoublePlane: \n";
    plane1.print();
    plane2.print();
  }
  void scale(float scaleFactor){
   plane1.scale(scaleFactor); 
   plane2.scale(scaleFactor);
  }
};

struct TriplePlane {
  Layer3D plane1;
  Layer3D plane2;
  Layer3D plane3;
  void print(){
    std::cout << "TriplePlane: \n";
    plane1.print();
    plane2.print();
    plane3.print();
  }
  void scale(float scaleFactor){
   plane1.scale(scaleFactor); 
   plane2.scale(scaleFactor);
   plane3.scale(scaleFactor);
  }
};

struct Crot3 {
  Axis3D axis1;
  Layer3D plane1;
  Layer3D plane2;
  Layer3D plane3;
  void print(){
    std::cout << "C3: \n";
    axis1.print();
    plane1.print();
    plane2.print();
    plane3.print();
  }
  void scale(float scaleFactor){
   axis1.scale(scaleFactor);
   plane1.scale(scaleFactor); 
   plane2.scale(scaleFactor);
   plane3.scale(scaleFactor);
  }
};


//For Gripper
struct Constraint {
  ConstraintType constraintType;
  boost::variant< std::vector<Eigen::VectorXf>, std::vector<Eigen::VectorXi> > jointConfigurations;
  void print(){
   std::cout << "Constraint:\n"; 
   std::cout << "Constraint Type: " << constraintType << std::endl;
   std::vector<Eigen::VectorXf> jointsFloat;
   std::vector<Eigen::VectorXi> jointsInt;
   switch (constraintType)
    {
    case BOOL: 
    {
      jointsInt = boost::get<std::vector<Eigen::VectorXi> > (jointConfigurations);
      for(int i = 0; i < jointsInt.size(); i++){
	std::cout << "Joint Configuration " << i << ":" << jointsInt.at(i).transpose() << "\n";
      }
    }
    break;
    case CONTINUOUS:
    {
      jointsFloat = boost::get<std::vector<Eigen::VectorXf> > (jointConfigurations);
      for(int i = 0; i < jointsFloat.size(); i++){
	std::cout << "Joint Configuration " << i << ":" << jointsFloat.at(i).transpose() << "\n";
      }
    }
    break;
    default: 
      std::cout << "[Error] Constraintsymmetry not correctly defined, exiting!\n";
      exit(EXIT_FAILURE);
      break;
    }  
  }
}; //Here only type definition

struct GripperSymmetry {
  
  SymmetryType symmetryType;  
  std::vector<Constraint> symmetryConstraint; //To Symmetry Constraint
  boost::variant<SinglePlane, Crot3> symmetryData;
  
  void print() {
    SinglePlane singlePlane;
    Crot3 c3Data;
    std::cout << "Symmetry Info:" << std::endl;
    std::cout << "Symmetry Type: " << symmetryType << std::endl;
    for (int i =0; i < symmetryConstraint.size(); i++) 
    {
      std::cout << "Symmetry Constraint at position : " << i << "\n";
      symmetryConstraint.at(i).print();
    }
    switch(symmetryType)
    {
      case C3:
      {
	c3Data = boost::get<Crot3> (symmetryData);
	c3Data.print();
      }
      break;
      case SINGLEPLANE:
      {
	singlePlane = boost::get<SinglePlane> (symmetryData);
	singlePlane.print();
      }
      break;
      default: 
      std::cout << "[Error] Symmetrydata not correctly defined, exiting!\n";
      exit(EXIT_FAILURE);
      break;
    }  
      
  }
};

//For non-Object, only 
// union SymmetryData {
//   Axial axial;
//   AxialSinglePlane axialSinglePlane;
//   SinglePlane singlePlane;
//   DoublePlane doublePlane;
//   TriplePlane triplePlane;
// };

//typdef  SymmetryData;


struct ObjectSymmetry {
  SymmetryType symmetryType;
  boost::variant<Axial, AxialSinglePlane, SinglePlane, DoublePlane, TriplePlane> symmetryData;  
};

#endif
