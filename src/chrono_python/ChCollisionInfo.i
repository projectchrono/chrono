%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionInfo.h"
#include "collision/ChCCollisionModel.h"

using namespace collision;

%}

// Forward ref
//%import "ChCollisionModel.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../collision/ChCCollisionInfo.h"    


