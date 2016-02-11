%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkRevolute.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Define also the shared pointer 
%shared_ptr(chrono::ChLinkRevolute)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkRevolute.h"  







