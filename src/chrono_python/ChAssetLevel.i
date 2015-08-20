%{

/* Includes the header in the wrapper code */
#include "assets/ChAssetLevel.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../assets/ChAssetLevel.h"    

// Define also the shared pointer 

%DefChSharedPtr(chrono::,ChAssetLevel)

