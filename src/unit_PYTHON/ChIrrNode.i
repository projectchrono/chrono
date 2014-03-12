%{

/* Includes the header in the wrapper code */
#include "irrlicht_interface/ChIrrNode.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../irrlicht_interface/ChIrrNode.h"    

// Define also the shared pointer chrono::ChShared<ChBoxShape> 
// (renamed as 'ChBoxShapeShared' in python)

// %DefChSharedPtr(chrono::,ChXxxxx)

