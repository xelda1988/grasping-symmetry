#include "GraspSymmetrizer/symmetry.h"

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