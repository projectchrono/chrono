%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkLinActuator.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChLinkLinActuator)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkLinActuator.h"  







