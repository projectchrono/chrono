%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBodyFrame.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChBodyFrame)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBodyFrame.h"    


