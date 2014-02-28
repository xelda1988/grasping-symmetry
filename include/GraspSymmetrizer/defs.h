#ifndef DEFS_H_
#define DEFS_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

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


#endif
