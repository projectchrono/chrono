%{

/* Includes the header in the wrapper code */
#include "chrono/fea/ChMesh.h"

using namespace chrono;

%}

%shared_ptr(chrono::fea::ChMesh)

/* Parse the header file to generate wrappers */
 %include "../../../chrono/fea/ChMesh.h"    


