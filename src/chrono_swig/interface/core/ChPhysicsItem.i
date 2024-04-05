%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChPhysicsItem.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChPhysicsItem)


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChPhysicsItem.h"    

