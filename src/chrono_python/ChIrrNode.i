%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrNode.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../chrono_irrlicht/ChIrrNode.h"    

// Define also the shared pointer chrono::ChShared<ChBoxShape> 
// (renamed as 'ChBoxShapeShared' in python)

// %DefChSharedPtr(chrono::,ChXxxxx)

