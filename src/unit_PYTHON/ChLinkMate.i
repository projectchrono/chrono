%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMate.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChLinkMate.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(ChLinkMateShared, ChLinkMate)
%DefChSharedPtr(ChLinkMateGenericShared, ChLinkMateGeneric)
%DefChSharedPtr(ChLinkMatePlaneShared, ChLinkMatePlane)
%DefChSharedPtr(ChLinkMateCoaxialShared, ChLinkMateCoaxial)
%DefChSharedPtr(ChLinkMateSphericalShared, ChLinkMateSpherical)
%DefChSharedPtr(ChLinkMateXdistanceShared, ChLinkMateXdistance)
%DefChSharedPtr(ChLinkMateParallelShared, ChLinkMateParallel)
%DefChSharedPtr(ChLinkMateOrthogonalShared, ChLinkMateOrthogonal)


