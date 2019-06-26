%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLink.h"

%}
 
%shared_ptr(chrono::ChLink)

// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLink.h"  







