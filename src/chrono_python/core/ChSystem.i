%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

using namespace chrono;

%}

%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChSystem::CustomCollisionCallback)

// Forward ref
%import "chrono_python/core/ChAssembly.i"
%import "chrono_python/core/ChTimestepper.i"
//%import "chrono_python/core/ChSolver.i"
%import "chrono_python/core/ChCollisionModel.i"
%import "chrono_python/core/ChCollisionInfo.i"

// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") CustomCollisionCallback;

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChSystem.h" 





