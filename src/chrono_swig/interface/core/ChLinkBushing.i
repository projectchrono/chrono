%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkBushing.h" 

%}

// Tell Swig about the Matrix66d template
%include "ChMatrix.i"


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChLinkBushing.h"  







