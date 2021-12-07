%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRSDA.h"

using namespace chrono;
%}

%shared_ptr(chrono::ChLinkRSDA)
%shared_ptr(chrono::ChLinkRSDA::TorqueFunctor)
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

/* Parse the header file to generate wrappers */
%feature("director") TorqueFunctor;
%include "../../../chrono/physics/ChLinkRSDA.h"
