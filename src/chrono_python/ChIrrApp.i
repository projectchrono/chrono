%{

/* Includes the header in the wrapper code */
#include "unit_IRRLICHT/ChIrrApp.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../unit_IRRLICHT/ChIrrApp.h"    

// Define also the shared pointer chrono::ChShared<ChXxxxx> 
// (renamed as 'ChXxxxxShared' in python)

// %DefChSharedPtr(chrono::,ChXxxxx)

