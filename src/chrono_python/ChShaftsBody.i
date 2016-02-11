%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsBody.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChShaftsBody)

/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsBody.h"  







