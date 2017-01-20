%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/collision/ChCCollisionModel.h"

using namespace collision;

%}

// Forward ref
//%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%include "../chrono/collision/ChCCollisionInfo.h"    


