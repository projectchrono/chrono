%{

/* Includes the header in the wrapper code */
#include "physics/ChShaft.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::postprocess::ChShaft)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChShaft.h"  







