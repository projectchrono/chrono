%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoadContainer.h"

%}
 
 %shared_ptr(chrono::ChLoadContainer)

// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLoadContainer.h"  







