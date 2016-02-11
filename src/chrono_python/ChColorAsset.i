%{

/* Includes the header in the wrapper code */
#include "assets/ChColorAsset.h"

using namespace chrono;

%}

// Enable shared pointer 
%shared_ptr(chrono::ChColorAsset)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChColorAsset.h"    



