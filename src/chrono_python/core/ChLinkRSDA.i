%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkRotSpringCB.h"

using namespace chrono;

// NESTED CLASSES - trick - step 1
//
// Trick for having a SWIG-specific global class that represent
// a nested class (nested are not supported in SWIG so this is a workaround)
// The workaround is done in various 1,2,3,.. steps. 
//
// STEP 1: do aliases for the c++ compiler when compiling the .cxx wrapper 

// for this nested class, inherit stubs (not virtual) as outside class
class TorqueFunctorP : public chrono::ChLinkRotSpringCB::TorqueFunctor {
public:
    TorqueFunctorP() {}
    virtual ~TorqueFunctorP() {}
    
    /// Calculate and return the general spring-damper force at the specified configuration.
    virtual double operator()(double time,             ///< current time
                              double angle,            ///< relative angle of rotation
                              double vel,              ///< relative angular speed
                              ChLinkRotSpringCB* link  ///< back-pointer to associated link
    ) {
      return 0;
    }
};

%}

%shared_ptr(chrono::ChLinkRotSpringCB)

 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") TorqueFunctorP;

// NESTED CLASSES - trick - step 2
//
// STEP 2: Now the nested classes  MyOutClass::MyNestedClass are declared  
// as ::MyNestedClass (as non-nested), for SWIG interpreter _only_:

class TorqueFunctorP {
public:
   TorqueFunctorP() {}
   virtual ~TorqueFunctorP() {}
   virtual double operator()(double time,             ///< current time
                             double angle,            ///< relative angle of rotation
                             double vel,              ///< relative angular speed
                             ChLinkRotSpringCB* link  ///< back-pointer to associated link
   ) {
                             return 0;
   }
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

%extend chrono::ChLinkRotSpringCB
{
   void RegisterTorqueFunctor(::TorqueFunctorP* functor)
   {
      $self->RegisterTorqueFunctor(functor);
   }
};

// NESTED CLASSES - trick - step 5
//
// STEP 5: note that if you override some functions by %extend, now you must deactivate the 
// original ones in the .h file, by using %ignore. NOTE that this must happen AFTER the %extend,
// and BEFORE the %include !!!

%ignore chrono::ChLinkRotSpringCB::RegisterTorqueFunctor();


/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChLinkRotSpringCB.h"
