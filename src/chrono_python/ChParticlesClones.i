%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChParticlesClones.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
//#define ChApi 

// Forward ref
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChParticlesClones.h"  





