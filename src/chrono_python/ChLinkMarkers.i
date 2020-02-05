%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkMarkers.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLink.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkMarkers.h"  







