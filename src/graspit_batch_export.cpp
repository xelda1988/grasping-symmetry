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
 

  string graspitRootPath = "/home/alexander/workspace/GraspSymmetrizer/resources/graspit/";
  //int i = 1; //testint
  
  ObjectDatabase objectDb("../resources/ObjectDatabase.xml");
  
  vector<string> objects;
  for(int i = 0; i < objectDb.objectDb_.size(); i++)
  objects.push_back(objectDb.objectDb_.at(i).getObjectName());
  
  //using naming conventions to create the filepaths!  
  vector< vector <string> > paths;
  vector< string > layout_paths;
  vector< string > path_per_object;
  vector< Object > objects_in_use;
    
  int fileCounter=1;
  for (int i = 0; i < objects.size(); i++)
  {
    while (true)
    {
      
      stringstream ss;
      ss << graspitRootPath << objects.at(i) << "_" << fileCounter << "_grasp_graspit.xml";
      
      if( ! file_exists(ss.str()) ) break;
      
      
      path_per_object.push_back(ss.str());
// cout << "[Debug]: 57" << ss.str() << endl;
      if(fileCounter==1){
	layout_paths.push_back(ss.str());
        objects_in_use.push_back(objectDb.objectDb_.at(i));
      }
      
      fileCounter++;
     
    }
    
    if(path_per_object.size() > 0) paths.push_back(path_per_object);
    path_per_object.resize(0);
    fileCounter=1;    
  }
  


  //Import actual grasps:
  Gripper gripper("/home/alexander/workspace/GraspSymmetrizer/resources/SchunkDexHandConfig.xml");
  
  //TODO: for the loop we need map from object to object_files
  //we need also map from object to layout_file
  //we need loop over all ouput grasps -> change name of paths to _gen1...
// cout << "[Debug]: 79: paths.size" << paths.size() << endl;
  
  for (int i = 0; i < paths.size(); i++){
    
// cout << "[Debug]: 79: paths(i).size " << paths.at(i).size() << endl;

    for (int j = 0; j < paths.at(i).size(); j++){
      
// cout << "[Debug]: 85: Actual grasp path:" << paths.at(i).at(j) << endl;

      string actualPath = paths.at(i).at(j);

      Grasp actualGrasp( actualPath );
    
      SymmetryOperation actualSymOperation(gripper, objects_in_use.at(i), actualGrasp );

      actualSymOperation.computeSymmetries();
      
      GraspDatabase graspDbOut = actualSymOperation.getGraspDb();
//       cout << "[Debug]: 80: Actual generated grasp Database:" << endl;
      graspDbOut.printGraspDatabase();
      
      //Save To XML
      graspDbOut.saveToXml("/home/alexander/workspace/GraspSymmetrizer/resources/GraspDatabase_intellact.xml");
      
      //Batch export of grasp Db
      
      for (int k = 0; k < graspDbOut.graspDb_.size(); k++){
	
	stringstream path_gen;
	string pathMod = actualPath.substr (0,actualPath.size()-3);
	path_gen << pathMod << "_gen_" << k << ".xml";
	graspDbOut.graspDb_.at(k).saveToGraspItXml(layout_paths.at(i), path_gen.str() );
      }
    
    
    }
  }
  return 0;
}

