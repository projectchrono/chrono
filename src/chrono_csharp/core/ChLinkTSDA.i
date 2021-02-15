// Include the C++ header(s)
%{
#include "chrono/physics/ChLinkTSDA.h"

using namespace chrono;
%}

%shared_ptr(chrono::ChLinkTSDA)
%shared_ptr(chrono::ChLinkTSDA::ForceFunctor)

// Tell SWIG about parent class
%import "ChLink.i"

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChLinkTSDA.h"
