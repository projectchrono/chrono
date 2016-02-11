%{

/* Includes the header in the wrapper code */
#include "physics/ChMaterialSurfaceBase.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChMaterialSurfaceBase)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChMaterialSurfaceBase.h"    


