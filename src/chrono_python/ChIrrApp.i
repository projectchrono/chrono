%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../chrono_irrlicht/ChIrrApp.h"    

// Define also the shared pointer chrono::ChShared<ChXxxxx> 
// (renamed as 'ChXxxxxShared' in python)

// %DefChSharedPtr(chrono::,ChXxxxx)

