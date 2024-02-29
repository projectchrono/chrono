%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkLockGear.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChLinkLockGear.h"
