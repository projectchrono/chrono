%{

/* Includes the header in the wrapper code */
#include "physics/ChContactContainerBase.h"

using namespace collision;

%}

// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") chrono::ChReportContactCallback;
%feature("director") chrono::ChAddContactCallback;

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChContactContainerBase.h"    

