%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

using namespace chrono;

// NESTED CLASSES - trick - step 1
//
// Trick for having a SWIG-specific global class that represent
// a nested class (nested are not supported in SWIG so this is a workaround)
// The workaround is done in various 1,2,3,.. steps. 
//
// STEP 1: do aliases for the c++ compiler when compiling the .cxx wrapper 

// for this nested class, inherit stubs (not virtual) as outside class
class ChCustomCollisionCallbackP : public chrono::ChSystem::CustomCollisionCallback {
	public: 
		ChCustomCollisionCallbackP() {}
		virtual ~ChCustomCollisionCallbackP() {}
		virtual void OnCustomCollision(chrono::ChSystem* msys) {
		    GetLog() << "You must implement OnCustomCollision()!\n";
		}
};

%}

%shared_ptr(chrono::ChSystem)

// Forward ref
%import "ChAssembly.i"
%import "ChTimestepper.i"
%import "ChCollisionModel.i"
%import "ChCollisionInfo.i"

// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") ChCustomCollisionCallbackP;

// NESTED CLASSES - trick - step 2
//
// STEP 2: Now the nested classes  MyOutClass::MyNestedClass are declared  
// as ::MyNestedClass (as non-nested), for SWIG interpreter _only_:

class ChCustomCollisionCallbackP {
  public:
    virtual ~ChCustomCollisionCallbackP() {}
    virtual void OnCustomCollision(chrono::ChSystem* msys) {}
};

// NESTED CLASSES - trick - step 3
//
// STEP 3: if needed, extend some functions of the 'un-nested' classes

// NESTED CLASSES - trick - step 4
//
// STEP 4: if needed, extend some functions of the class that hosted the nested classes,
// because for example they returned a refrence to nested chrono::ChSystem::IteratorBody and
// this will confuse the python runtime, so do override these functions so that they return 
// a ::IteratorBody type (see step 2), that will be interpreted ok in the python runtime.

%extend chrono::ChSystem
{
	void RegisterCustomCollisionCallback(::ChCustomCollisionCallbackP* mcallb)  // note the :: at the beginning
	  {
		  $self->RegisterCustomCollisionCallback(mcallb);
	  }
};

// NESTED CLASSES - trick - step 5
//
// STEP 5: note that if you override some functions by %extend, now you must deactivate the 
// original ones in the .h file, by using %ignore. NOTE that this must happen AFTER the %extend,
// and BEFORE the %include !!!

%ignore chrono::ChSystem::RegisterCustomCollisionCallback();


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChSystem.h" 





