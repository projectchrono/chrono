%{

/* Includes the header in the wrapper code */
#include "assets/ChCylinderShape.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../assets/ChCylinderShape.h"    

// Define also the shared pointer 

%DefChSharedPtr(chrono::,ChCylinderShape)

