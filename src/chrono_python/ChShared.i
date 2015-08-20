%{

/* Includes the header in the wrapper code */
#include "core/ChShared.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../core/ChShared.h"    

