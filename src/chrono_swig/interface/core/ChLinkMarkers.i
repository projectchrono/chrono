#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChLinkMarkers::GetFrame1Rel "public override"
%csmethodmodifiers chrono::ChLinkMarkers::GetFrame2Rel "public override"

#endif             // --------------------------------------------------------------------- CSHARP



%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkMarkers.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLink.i"


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChLinkMarkers.h"  







