#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "GraspSymmetrizer/defs.h"
#include "GraspSymmetrizer/grasp.h"
#include "GraspSymmetrizer/gripper.h"
#include "GraspSymmetrizer/object.h"
#include "GraspSymmetrizer/symmetry.h"
#include "GraspSymmetrizer/util.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

inline bool file_exists (const std::string& name) {
    ifstream f(name.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }   
}

int main () {
 
  bool fileExist = true;
  string graspitRootPath = "/home/alexander/workspace/GraspSymmetrizer/resources/graspit/";
  //int i = 1; //testint
  
  ObjectDatabase objectDb("../resources/ObjectDatabase.xml");
  
  vector<string> objects;
  for(int i = 0; i < objectDb.objectDb_.size(); i++)
  objects.push_back(objectDb.objectDb_.at(i).getObjectName());
  
  //using naming conventions to create the filepaths!  
  vector<string> paths;
  vector<string> layout_paths;
  
  int fileCounter=1;
  for (int i = 0; i < objects.size(); i++)
  {
    while (fileExist)
    {
      
      stringstream ss;
      ss << graspitRootPath << objects.at(i) << "_" << fileCounter << "_grasp_graspit.xml";
      fileExist=file_exists(ss.str());
      paths.push_back(ss.str());
      if(fileCounter==1) layout_paths.push_back(ss.str());
    
    }    
    fileCounter=1;    
  }
  
  //Import actual grasps:
  Gripper gripper("/home/alexander/workspace/GraspSymmetrizer/resources/SchunkDexHandConfig.xml");
  
  //TODO: for the loop we need map from object to object_files
  //we need also map from object to layout_file
  //we need loop over all ouput grasps -> change name of paths to _gen1...
  for (int i = 0; i < paths.size(); i++){
    
    Grasp actualGrasp(paths.at(i).c_str());
    
    SymmetryOperation actualSymOperation(gripper, objectDb.objectDb_.at(0), actualGrasp );

    actualSymOperation.computeSymmetries();
    
    GraspDatabase graspDbOut = actualSymOperation.getGraspDb();
    graspDbOut.printGraspDatabase();
    graspDbOut.graspDb_.at(1).saveToGraspItXml(layout_paths.at(0), paths.at(0) );
  }
  return 0;
}