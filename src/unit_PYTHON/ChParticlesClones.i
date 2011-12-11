%{

/* Includes the header in the wrapper code */
#include "physics/ChParticlesClones.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Forward ref
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%include "../physics/ChParticlesClones.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(ChParticlesClonedShared, ChParticlesClones)

