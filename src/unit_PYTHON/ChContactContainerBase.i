%{

/* Includes the header in the wrapper code */
#include "physics/ChContactContainerBase.h"

using namespace collision;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") chrono::ChReportContactCallback;
%feature("director") chrono::ChAddContactCallback;


/* Parse the header file to generate wrappers */
%include "../physics/ChContactContainerBase.h"    

