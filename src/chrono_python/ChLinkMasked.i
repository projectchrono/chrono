%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkMasked.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"



/* Parse the header file(s) to generate wrappers */
%include "../chrono/physics/ChLinkForce.h"
%include "../chrono/physics/ChLinkMasked.h"







