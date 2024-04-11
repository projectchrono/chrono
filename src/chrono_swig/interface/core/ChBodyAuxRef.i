%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBodyAuxRef.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChContactable) 
%shared_ptr(chrono::ChContactable_1vars<6>)
%shared_ptr(chrono::ChBodyAuxRef)

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChBodyAuxRef.h"    

