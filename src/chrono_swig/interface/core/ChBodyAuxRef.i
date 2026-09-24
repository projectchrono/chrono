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
// SetCoordsys / SetCoordsysDt override ChFrame / ChFrameMoving members. C# has no multiple
// inheritance, so SWIG drops the ChBodyFrame base and - unlike the setters above, which ChBody
// re-exposes - these are in no C# base class here: "override" has nothing to override and
// "new" nothing to hide.
%csmethodmodifiers chrono::ChBodyAuxRef::SetCoordsys "public"
%csmethodmodifiers chrono::ChBodyAuxRef::SetCoordsysDt "public"
#endif             // --------------------------------------------------------------------- CSHARP

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"
%include "../../../chrono/physics/ChBodyAuxRef.h"    

