// Ensure that generated C# code does not use 'override' for ChBodyFrame and ChLoadableUVW virtual methods implemented by ChBody.
// This is because ChBody uses multiple inheritance and SWIG ignores all but the first base class.

%csmethodmodifiers chrono::ChBody::Variables "public"
%csmethodmodifiers chrono::ChBody::LoadableGetVariables "public"
%csmethodmodifiers chrono::ChBody::LoadableStateIncrement "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlock_x "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlock_w "public"
%csmethodmodifiers chrono::ChBody::ComputeNF "public"


%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChBody.h"

%}
 
%shared_ptr(chrono::ChBody)

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChBody.h"  




