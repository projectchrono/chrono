%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChObject.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChObj)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChObject.h"    

