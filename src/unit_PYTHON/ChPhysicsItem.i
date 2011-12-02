%{

/* Includes the header in the wrapper code */
#include "physics/ChPhysicsItem.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../physics/ChPhysicsItem.h"    
