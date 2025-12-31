%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChParticleCloud.h"

%}


// Undefine ChApi otherwise SWIG gives a syntax error
//#define ChApi 

// Forward ref
%import "ChContactMaterial.i"
%import "ChCollisionModel.i"

%shared_ptr(chrono::ChParticleCloud)
%shared_ptr(chrono::ChParticleCloud::VisibilityCallback)


/* Parse the header file to generate wrappers */
%ignore chrono::ChParticle;
%include "../../../chrono/physics/ChParticleCloud.h"
