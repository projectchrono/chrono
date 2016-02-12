%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionInfo.h"
#include "collision/ChCCollisionModel.h"

using namespace collision;

%}

// Forward ref
//%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%include "../chrono/collision/ChCCollisionInfo.h"    


