%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRotSpringCB.h"

using namespace chrono;
%}

%shared_ptr(chrono::ChLinkRotSpringCB)
%shared_ptr(chrono::ChLinkRotSpringCB::TorqueFunctor)
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

/* Parse the header file to generate wrappers */
%feature("director") TorqueFunctor;
%include "../../chrono/physics/ChLinkRotSpringCB.h"
