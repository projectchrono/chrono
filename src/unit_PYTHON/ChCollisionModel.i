%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionModel.h"

using namespace collision;

%}


// Forward ref
//%import "ChXxxxx.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%feature("director") chrono::collision::ChCollisionModel;
%include "../collision/ChCCollisionModel.h"

