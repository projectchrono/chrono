%{

/* Includes the header in the wrapper code */
#include "irrlicht_interface/ChIrrAppInterface.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../unit_IRRLICHT/ChIrrAppInterface.h"    

// Define also the shared pointer chrono::ChShared<ChXxxxx> 
// (renamed as 'ChXxxxxShared' in python)

// %DefChSharedPtr(chrono::ChXxxxx)

