%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChParticleCloud.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
//#define ChApi 

// Forward ref
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"


/* Parse the header file to generate wrappers */
%ignore chrono::ChAparticle;
%include "../../../chrono/physics/ChParticleCloud.h"
