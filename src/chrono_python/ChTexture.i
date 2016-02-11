%{

/* Includes the header in the wrapper code */
#include "assets/ChTexture.h"

using namespace chrono;

%}

// Enable shared pointer 
%shared_ptr(chrono::ChTexture)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChTexture.h"    



