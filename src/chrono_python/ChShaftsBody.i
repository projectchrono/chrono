%{

/* Includes the header in the wrapper code */
#include "physics/ChShaftsBody.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"


/* Parse the header file to generate wrappers */
%include "../physics/ChShaftsBody.h"  







