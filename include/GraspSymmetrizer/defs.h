#ifndef GRASPSYMMETRIZER_DEFS_H_
#define GRASPSYMMETRIZER_DEFS_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

//Simple Types
typedef Eigen::Vector3f Point3D;
typedef Eigen::Matrix4f Pose;

struct Axis3D {
  Point3D point1;
  Point3D point2;
};

struct Layer3D {
  Point3D point1;
  Point3D point2;
  Point3D point3;
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
}

struct Axial {
  Axis3D axis1;
};

struct AxialSinglePlane {
  Axis3D axis1;
  Layer3D layer1;  
};

struct SinglePlane {
  Layer3D plane1;
};

struct DoublePlane {
  Layer3D plane1;
  Layer3D plane2;
  Layer3D plane3;
};

struct TriplePlane {
  Layer3D plane1;
  Layer3D plane2;
  Layer3D plane3;
};


//For Gripper
struct Constraint {
  ConstraintType constraintType;
  std::vector<Eigen::MatrixXf> jointConfigurations;
}; //Here only type definition

struct GripperSymmetry {
  SymmetryType symmetryType;
  std::vector<Constraint> symmmetryData;
}

//For Object
union SymmetryData {
  Axial axial;
  AxialSinglePlane axialSinglePlane;
  SinglePlane singlePlane;
  DoublePlane doublePlane;
  TriplePlane triplePlane;
}

struct ObjectSymmetry {
  SymmetryType symmetryType;
  SymmetryData symmetryData;  
}

#endif
