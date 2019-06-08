%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBodyAuxRef.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChBodyAuxRef)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBodyAuxRef.h"    

