#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChBodyAuxRef::SetPos "public"
%csmethodmodifiers chrono::ChBodyAuxRef::SetRot "public"
%csmethodmodifiers chrono::ChBodyAuxRef::SetCoordsys "public"

#endif             // --------------------------------------------------------------------- CSHARP

%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBodyAuxRef.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChContactable) 
%shared_ptr(chrono::ChContactable_1vars<6>)
%shared_ptr(chrono::ChBodyAuxRef)

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"
%include "../../../chrono/physics/ChBodyAuxRef.h"    

