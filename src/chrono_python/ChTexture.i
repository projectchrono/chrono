%{

/* Includes the header in the wrapper code */
#include "assets/ChTexture.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChTexture.h"    

// Define also the shared pointer 

%DefChSharedPtr(chrono::,ChTexture)

