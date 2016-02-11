%{

/* Includes the header in the wrapper code */
#include "assets/ChCamera.h"

using namespace chrono;

%}

// Enable shared pointer 
%shared_ptr(chrono::ChCamera)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChCamera.h"    



