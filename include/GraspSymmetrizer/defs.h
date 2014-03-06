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
};

//Generic Symmetries
enum SymmetryType { 
  SINGLEPLANE = 1,
  DOUBLEPLANE, 
  TRIPLEPLANE,
  AXIAL,
  AXIALSINGLEPLANE,
  C3 //Hand all 120degree rotation
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
};

struct AxialSinglePlane {
  Axis3D axis1;
  Layer3D plane1;
  void print(){
    std::cout << "AxialSinglePlane: \n";
    axis1.print();
    plane1.print();
  }
};

struct SinglePlane {
  Layer3D plane1;
  void print(){
    std::cout << "SinglePlane: \n";
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
};


//For Gripper
struct Constraint {
  ConstraintType constraintType;
  std::vector<Eigen::MatrixXf> jointConfigurations;
}; //Here only type definition

struct GripperSymmetry {
  SymmetryType symmetryType;
  std::vector<Constraint> symmmetryData;
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
