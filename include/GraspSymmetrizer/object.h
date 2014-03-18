#ifndef GRASPSYMMETRIZER_OBJECT_H_
#define GRASPSYMMETRIZER_OBJECT_H_

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "defs.h"
#include "../tinyxml2/tinyxml2.h"


class Object {
  
  int objectID_;
  std::string objectName_;
  ObjectSymmetry objectSymmetry_; //see defs.h for type
  
public:
  
  void loadFromXml(tinyxml2::XMLElement* objectElement); //Pointer to Element of XMLfile
  void printObjectInfo();
  ObjectSymmetry getObjectSymmetry() const{
    return objectSymmetry_;
  }
  
  //Constructors
  Object(tinyxml2::XMLElement* objectElement) {
    loadFromXml(objectElement);
  }
  //destructor
  //~Object();
};

class ObjectDatabase {
  
  
  
public:
  
  std::vector<Object> objectDb_;
  void loadFromXml(const std::string filePath);
  void printObjectDatabase();

  
  ObjectDatabase (const std::string filePath){
  
    loadFromXml(filePath);
  }
  ~ObjectDatabase(){}
};
  
#endif
