%{

/* Includes the header in the wrapper code */
#include "physics/ChProximityContainerBase.h"

using namespace collision;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") chrono::ChAddProximityCallback;
%feature("director") chrono::ChReportProximityCallback;


/* Parse the header file to generate wrappers */
%include "../physics/ChProximityContainerBase.h"    

