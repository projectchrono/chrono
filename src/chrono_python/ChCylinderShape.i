%{

/* Includes the header in the wrapper code */
#include "assets/ChCylinderShape.h"

using namespace chrono;

%}

// Enable shared pointer
%shared_ptr(chrono::ChCylinderShape)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChCylinderShape.h"    



