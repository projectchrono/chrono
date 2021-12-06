%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRSDA.h"

using namespace chrono;

%}

//%feature("director") chrono::ChLinkRSDA::TorqueFunctor;

%shared_ptr(chrono::ChLinkRSDA)
%shared_ptr(chrono::ChLinkRSDA::TorqueFunctor)
 
// Tell SWIG about parent class
%import "ChLinkMarkers.i"

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChLinkRSDA.h"
