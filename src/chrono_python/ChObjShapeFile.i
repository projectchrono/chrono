%{

/* Includes the header in the wrapper code */
#include "assets/ChObjShapeFile.h"

using namespace chrono;

%}

// Enable shared pointer
%shared_ptr(chrono::ChObjShapeFile)

/* Parse the header file to generate wrappers */
%include "../chrono/assets/ChObjShapeFile.h"    



