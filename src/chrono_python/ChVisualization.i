%{

/* Includes the header in the wrapper code */
#include "assets/ChVisualization.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChVisualization.h"    


// Define also the shared pointer chrono::ChShared<ChVisualization> 
// (renamed as 'ChVisualizationShared' in python)

%DefChSharedPtr(chrono::,ChVisualization)
