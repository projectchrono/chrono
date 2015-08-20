%{

/* Includes the header in the wrapper code */
#include "assets/ChSphereShape.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChSphereShape.h"    

// Define also the shared pointer chrono::ChShared<ChBoxShape> 
// (renamed as 'ChBoxShapeShared' in python)

%DefChSharedPtr(chrono::,ChSphereShape)

