%{

/* Includes the header in the wrapper code */
#include "physics/ChMaterialSurface.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../physics/ChMaterialSurface.h"    


// Define also the object for the shared pointer
%include "../core/ChSmartpointers.h"
//%extend_smart_pointer(chrono::ChSharedPtr<ChMaterialSurface>);
%template(ChSharedMaterialSurface) chrono::ChSharedPtr<ChMaterialSurface>;