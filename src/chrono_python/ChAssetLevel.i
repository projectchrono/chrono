%{

/* Includes the header in the wrapper code */
#include "assets/ChAssetLevel.h"

using namespace chrono;

%}

// Enable shared pointer 
%shared_ptr(chrono::ChAssetLevel)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChAssetLevel.h"    



