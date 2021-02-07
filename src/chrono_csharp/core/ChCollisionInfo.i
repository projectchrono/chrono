%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/collision/ChCollisionModel.h"

using namespace collision;

%}

// Forward ref
//%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%include "../../chrono/collision/ChCollisionInfo.h"


