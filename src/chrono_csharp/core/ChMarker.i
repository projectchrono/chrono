%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChBody.h"
%}
 
namespace chrono { class ChBody; }

%shared_ptr(chrono::ChMarker)

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChMarker.h"  
