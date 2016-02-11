%{

/* Includes the header in the wrapper code */
#include "assets/ChBoxShape.h"

using namespace chrono;

%}

// Enable shared pointer
%shared_ptr(chrono::ChBoxShape)

/* Parse the header file to generate wrappers */
 %include "../chrono/assets/ChBoxShape.h"    



