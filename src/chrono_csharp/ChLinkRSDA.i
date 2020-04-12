%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRotSpringCB.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChLinkRotSpringCB)

 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkRotSpringCB.h"
