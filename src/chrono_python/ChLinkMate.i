%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMate.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChLinkMate)
%shared_ptr(chrono::ChLinkMateGeneric)
%shared_ptr(chrono::ChLinkMatePlane)
%shared_ptr(chrono::ChLinkMateCoaxial)
%shared_ptr(chrono::ChLinkMateSpherical)
%shared_ptr(chrono::ChLinkMateXdistance)
%shared_ptr(chrono::ChLinkMateParallel)
%shared_ptr(chrono::ChLinkMateOrthogonal)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkMate.h"  






