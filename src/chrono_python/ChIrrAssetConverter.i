%{

/* Includes the header in the wrapper code */
#include "unit_IRRLICHT/ChIrrAssetConverter.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../unit_IRRLICHT/ChIrrAssetConverter.h"    

// Define also the shared pointer chrono::ChShared<ChBoxShape> 
// (renamed as 'ChBoxShapeShared' in python)

// %DefChSharedPtr(chrono::, ChXxxxx)

