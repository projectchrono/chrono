%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkTSDA.h"

using namespace chrono;
%}

%shared_ptr(chrono::ChLinkTSDA)
%shared_ptr(chrono::ChLinkTSDA::ForceFunctor)
 
// Tell SWIG about parent class in Python
%import "ChLink.i"

/* Parse the header file to generate wrappers */
%feature("director") ForceFunctor;
%include "../../chrono/physics/ChLinkTSDA.h"
