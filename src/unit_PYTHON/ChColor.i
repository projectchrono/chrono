%{

/* Includes the header in the wrapper code */
#include "assets/ChColor.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../assets/ChColor.h"    

// Define also the shared pointer 

//not needed because not a shared ptr   
//%DefChSharedPtr(ChAssetLevelShared, ChAssetLevel)

