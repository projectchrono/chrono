%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkEngine.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"
// Forward ref (parent class does not need %import if all .i are included in proper order
%import "ChShaft.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkEngine.h"  







