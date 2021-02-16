%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkSpring.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"


/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChLinkSpring.h"  







