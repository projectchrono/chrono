#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChLinkRevoluteSpherical::GetFrame1Rel "public override"
%csmethodmodifiers chrono::ChLinkRevoluteSpherical::GetFrame2Rel "public override"

#endif             // --------------------------------------------------------------------- CSHARP

%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRevoluteSpherical.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChLinkRevoluteSpherical.h"  







