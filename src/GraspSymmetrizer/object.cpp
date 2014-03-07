#include "GraspSymmetrizer/object.h"
#include "GraspSymmetrizer/util.h"
#include <stdexcept>

void Object::loadFromXml(tinyxml2::XMLElement* objectElement) {
 
  std::vector<std::string> attributesObject, attributesSymmetry, attributesPoint, valuesObject, valuesSymmetry;
  std::vector<float> valuesPoint;
  
  attributesObject.push_back("ID");
  attributesObject.push_back("name");
  
  attributesSymmetry.push_back("type");
  
  attributesPoint.push_back("x");
  attributesPoint.push_back("y");
  attributesPoint.push_back("z");
  
  //getting attributes for object
  getAttributeList(valuesObject, attributesObject, objectElement);
  
  objectName_=valuesObject.at(1);

  objectID_=charArrayToInt(valuesObject.at(0).c_str());
  
  //getting attributes for object -only one symmetry per object  
  tinyxml2::XMLElement* symmetryPtr = objectElement->FirstChildElement("symmetry");
  
  if(symmetryPtr != NULL)
  {
    getAttributeList(valuesSymmetry, attributesSymmetry, symmetryPtr);
  }
  else
  {
    std::cout << "[Error] Wrong XML Element Name: symmetry" << std::endl;
    exit(EXIT_FAILURE);
  }
  //Mapping strings to enum type, see util
  getSymmetryTypeFromString(objectSymmetry_.symmetryType,valuesSymmetry.at(0));
  
  //getting attributes for the layers and stuff, several cases and element, looping
  
  tinyxml2::XMLElement* layerOrAxisPtr = symmetryPtr->FirstChildElement();
  tinyxml2::XMLElement* pointPtr = NULL;
  
  switch (objectSymmetry_.symmetryType)
  {
  case SINGLEPLANE: 
    {
      
      SinglePlane singlePlane; //actual single plane in the loop;
      int n =0;
      int m = 0;
      //Outer Loop
      for (; layerOrAxisPtr; layerOrAxisPtr=layerOrAxisPtr->NextSiblingElement() )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
//std::cout << "[Debug 50]:Outer Loop " << "\n" << std::flush;
//std::cout << "[Debug 50]:Pointer Value " << layerOrAxisPtr->Name() << "\n" << std::flush;
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr=pointPtr->NextSiblingElement() ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
//std::cout << "[Debug 60]:Inner Loop "  << "\n" << std::flush;
//std::cout << "[Debug 60]:Pointer Value " << pointPtr->Name() << "\n" << std::flush;
//std::cout << "[Debug 60]: m " << m << "\n" << std::flush;
//std::cout << "[Debug 60]: n " << n <<  "\n" << std::flush;
//std::cout << "[Debug 60]: ValuePoint(1) " << valuesPoint.at(1) << "\n" << std::flush;

	  if     (m==0) singlePlane.plane1.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==1) singlePlane.plane1.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==2) singlePlane.plane1.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      objectSymmetry_.symmetryData = singlePlane;
    }
    break;
  case DOUBLEPLANE: 
    {
      DoublePlane doublePlane; //actual single plane in the loop;
      int n =0;
      int m = 0;
      //Outer Loop
      for (; layerOrAxisPtr; layerOrAxisPtr =layerOrAxisPtr->NextSiblingElement("layer") )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
	
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr=pointPtr->NextSiblingElement("point") ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
	  if (n==0)
	  {	    
	    if     (m==0) doublePlane.plane1.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==1) doublePlane.plane1.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==2) doublePlane.plane1.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  }
	  else if (n==1){
	    if     (m==0) doublePlane.plane2.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==1) doublePlane.plane2.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==2) doublePlane.plane2.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);	    
	  }
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      objectSymmetry_.symmetryData = doublePlane;
    }
    break;
  case TRIPLEPLANE: 
    {
      TriplePlane triplePlane; //actual single plane in the loop;
      int n =0;
      int m = 0;
      //Outer Loop
      for (; layerOrAxisPtr; layerOrAxisPtr=layerOrAxisPtr->NextSiblingElement("layer") )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
	
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr=pointPtr->NextSiblingElement("point") ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
	  if (n==0)
	  {	    
	    if     (m==0) triplePlane.plane1.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==1) triplePlane.plane1.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==2) triplePlane.plane1.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  }
	  else if (n==1){
	    if     (m==0) triplePlane.plane2.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==1) triplePlane.plane2.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==2) triplePlane.plane2.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);	    
	  }
	  else if (n==2)
	  {
	    if     (m==0) triplePlane.plane3.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==1) triplePlane.plane3.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	    else if(m==2) triplePlane.plane3.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);	    
	  }
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      objectSymmetry_.symmetryData = triplePlane;
    }
    break;
  case AXIAL: 
    {
      Axial axial; //actual single plane in the loop;
      int n =0;
      int m = 0;
      //Outer Loop
      for (; layerOrAxisPtr; layerOrAxisPtr=layerOrAxisPtr->NextSiblingElement("axis") )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
	
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr = pointPtr->NextSiblingElement("point") ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
std::cout << "[Debug 60]: ValuePoint(1) " << valuesPoint.at(1) << "\n" << std::flush;
	  if     (m==0) axial.axis1.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==1) axial.axis1.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      objectSymmetry_.symmetryData = axial;
    }
    break;
  case AXIALSINGLEPLANE: 
    {
      AxialSinglePlane axialSinglePlane; //actual single plane in the loop;
      Layer3D layer;
      Axis3D axis;
      int n =0;
      int m = 0;
      //Outer Loop for axis
      layerOrAxisPtr = symmetryPtr->FirstChildElement("axis");
      for (; layerOrAxisPtr; layerOrAxisPtr=layerOrAxisPtr->NextSiblingElement("axis") )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
	
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr=pointPtr->NextSiblingElement("point") ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
	  
	  if     (m==0) axis.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==1) axis.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      
      //Outer Loop for layer
      n=0;
      m=0;
      layerOrAxisPtr = symmetryPtr->FirstChildElement("layer");
      for (; layerOrAxisPtr; layerOrAxisPtr=layerOrAxisPtr->NextSiblingElement("layer") )
      {
	pointPtr = layerOrAxisPtr->FirstChildElement("point");
	
	m=0;
	//Inner Loop - getting the full layer or axis
	for(; pointPtr; pointPtr=pointPtr->NextSiblingElement("point") ){
	  getAttributeList(valuesPoint, attributesPoint, pointPtr);
	  
	  if     (m==0) layer.point1 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==1) layer.point2 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  else if(m==2) layer.point3 << valuesPoint.at(0), valuesPoint.at(1), valuesPoint.at(2);
	  
	  m++;
	  valuesPoint.resize(0); //erasing content
	}
	//In Outer Loop 
	n++;
      }
      
      axialSinglePlane.axis1 = axis;
      axialSinglePlane.plane1 = layer;
      objectSymmetry_.symmetryData = axialSinglePlane;
    }
    break;
  default: 
    std::cout << "[Error] Objectsymmetry not correctly defined on XML, exiting!\n";
    exit(EXIT_FAILURE);
    break;
  }  
  
}

void Object::printObjectInfo(){
  
  std::cout << "Object Name: " << objectName_ << std::endl;
  std::cout << "Object ID: " << objectID_ << std::endl;
  std::cout << "Symmetry Type: " << objectSymmetry_.symmetryType << std::endl;
  std::cout << "Symmetry Info: \n";
  switch (objectSymmetry_.symmetryType)
  {
  case SINGLEPLANE: boost::get<SinglePlane>(objectSymmetry_.symmetryData).print();  break;
  case DOUBLEPLANE: boost::get<DoublePlane>(objectSymmetry_.symmetryData).print();  break;
  case TRIPLEPLANE: boost::get<TriplePlane>(objectSymmetry_.symmetryData).print();  break;
  case AXIAL: boost::get<Axial>(objectSymmetry_.symmetryData).print();  break;
  case AXIALSINGLEPLANE: boost::get<AxialSinglePlane>(objectSymmetry_.symmetryData).print();  break;
  default: 
    std::cout << "Objectsymmetry not correctly defined, exiting!\n";
    exit(EXIT_FAILURE);
    break;
  }  
}

void ObjectDatabase::loadFromXml(const std::string filePath){
  
  tinyxml2::XMLDocument doc;
  doc.LoadFile(filePath.c_str());
  
  if (!doc.Error()) {
    
    tinyxml2::XMLElement * xeObject = doc.FirstChildElement()->FirstChildElement("object");
    
    while(xeObject)
    {
      Object currentObject(xeObject);
      objectDb_.push_back(currentObject);
      xeObject = xeObject->NextSiblingElement("object");
    }
  }
  else {
  
    std::cout << "Document Parsing Error: " << filePath << "exiting" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void ObjectDatabase::printObjectDatabase(){
 
  std::cout << "Object Database:\n";
  for (int i = 0; i < objectDb_.size(); i++) {
    objectDb_.at(i).printObjectInfo();
  }
}