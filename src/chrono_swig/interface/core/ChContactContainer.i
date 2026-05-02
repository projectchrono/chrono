%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChContactContainer.h"
%}

%shared_ptr(chrono::ChContactContainer::ReportContactCallback)
%shared_ptr(chrono::ChContactContainer::AddContactCallback)
%shared_ptr(chrono::ChContactContainer)

// Forward ref
%import "ChCollisionModel.i"
%import "ChCollisionInfo.i"

// Cross-inheritance for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") chrono::ChContactContainer::ReportContactCallback;
%feature("director") chrono::ChContactContainer::AddContactCallback;

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChContactContainer.h"    
