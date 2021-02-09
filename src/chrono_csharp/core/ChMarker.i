%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChBody.h"
%}
 
namespace chrono { class ChBody; }

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChMarker.h"  
