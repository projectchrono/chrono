%{

/* Includes the header in the wrapper code */
#include "physics/ChMaterialSurface.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChMaterialSurface)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChMaterialSurface.h"    


