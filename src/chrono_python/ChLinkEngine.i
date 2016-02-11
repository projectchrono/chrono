%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkEngine.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"
// Forward ref (parent class does not need %import if all .i are included in proper order
%import "ChShaft.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChLinkEngine)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkEngine.h"  







