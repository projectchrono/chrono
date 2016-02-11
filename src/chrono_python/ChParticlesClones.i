%{

/* Includes the header in the wrapper code */
#include "physics/ChParticlesClones.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Forward ref
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"

// Enable shared pointer
%shared_ptr(chrono::ChParticlesClones)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChParticlesClones.h"  





