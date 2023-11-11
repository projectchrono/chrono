%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactable.h"

%}

%shared_ptr(chrono::ChContactContainer::ReportContactCallback)
%shared_ptr(chrono::ChContactContainer::AddContactCallback)
%shared_ptr(chrono::ChContactContainer)
%shared_ptr(chrono::ChContactable)

%inline %{
  chrono::ChBody* CastContactableToChBody(chrono::ChContactable* base) {
    chrono::ChBody* ptr_out = dynamic_cast<chrono::ChBody*>(base);
	if (ptr_out == NULL) {
        throw std::invalid_argument( "Wrong Upcast Choice" );
    }
    return ptr_out;
  }
%}

// Forward ref
%import "ChCollisionModel.i"
%import "ChCollisionInfo.i"

// Cross-inheritance for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") chrono::ChContactContainer::ReportContactCallback;
%feature("director") chrono::ChContactContainer::AddContactCallback;

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChContactable.h"
%include "../../../chrono/physics/ChContactContainer.h"    
