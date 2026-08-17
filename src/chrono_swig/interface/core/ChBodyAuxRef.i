%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBodyAuxRef.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChContactable) 
%shared_ptr(chrono::ChContactable_1vars)
%shared_ptr(chrono::ChBodyAuxRef)

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
// ensure SWIG knows how to handle base class overrides so we're not hiding members
%csmethodmodifiers chrono::ChBodyAuxRef::SetPos "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetPosDt "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetLinVel "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetRot "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetRotDt "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetAngVelLocal "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetAngVelParent "public new"
#endif             // --------------------------------------------------------------------- CSHARP

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"
%include "../../../chrono/physics/ChBodyAuxRef.h"    

