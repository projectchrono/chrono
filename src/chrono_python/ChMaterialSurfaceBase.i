%{

/* Includes the header in the wrapper code */
#include "physics/ChMaterialSurfaceBase.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChMaterialSurfaceBase.h"    


// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(chrono::,ChMaterialSurfaceBase)