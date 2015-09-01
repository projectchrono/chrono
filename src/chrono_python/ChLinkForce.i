%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkForce.h"

%}
 
// Tell SWIG about parent class in Python
//%import "..."



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkForce.h"



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

//%DefChSharedPtr(chrono::,ChLinkLimit)



