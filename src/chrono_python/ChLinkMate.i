%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMate.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkMate.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(chrono::,ChLinkMate)
%DefChSharedPtr(chrono::,ChLinkMateGeneric)
%DefChSharedPtr(chrono::,ChLinkMatePlane)
%DefChSharedPtr(chrono::,ChLinkMateCoaxial)
%DefChSharedPtr(chrono::,ChLinkMateSpherical)
%DefChSharedPtr(chrono::,ChLinkMateXdistance)
%DefChSharedPtr(chrono::,ChLinkMateParallel)
%DefChSharedPtr(chrono::,ChLinkMateOrthogonal)


