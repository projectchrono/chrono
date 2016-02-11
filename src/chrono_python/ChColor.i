%{

/* Includes the header in the wrapper code */
#include "assets/ChColor.h"

using namespace chrono;

%}

// Enable shared pointer 
%shared_ptr(chrono::ChColor)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChColor.h"    


