%{

/* Includes the header in the wrapper code */
#include "assets/ChBoxShape.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChBoxShape.h"    

// Define also the shared pointer chrono::ChShared<ChBoxShape> 
// (renamed as 'ChBoxShapeShared' in python)

%DefChSharedPtr(chrono::,ChBoxShape)

