%{

/* Includes the header in the wrapper code */
#include "irrlicht_interface/ChIrrApp.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../irrlicht_interface/ChIrrApp.h"    

// Define also the shared pointer chrono::ChShared<ChXxxxx> 
// (renamed as 'ChXxxxxShared' in python)

// %DefChSharedPtr(ChXxxxxShared, ChXxxxx)

