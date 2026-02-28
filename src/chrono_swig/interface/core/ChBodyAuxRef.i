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
// ensure SWIG knows how to handle base class overides so we're not hiding members (for Unity)
%csmethodmodifiers chrono::ChBodyAuxRef::SetPos "public new"
%csmethodmodifiers chrono::ChBodyAuxRef::SetRot "public new"

#endif             // --------------------------------------------------------------------- CSHARP

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"
%include "../../../chrono/physics/ChBodyAuxRef.h"    

