%{
#include "chrono/physics/ChLinkRSDA.h"

using namespace chrono;

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class ChTorqueFunctorP : public chrono::ChLinkRSDA::TorqueFunctor {
    public:
        ChTorqueFunctorP() {}
        virtual double evaluate(double time,
                                double angle,
                                double vel,
                                chrono::ChLinkRSDA* link) override {
            GetLog() << "You must implement the function evaluate()!\n";
            return 0.0;
        }
};

#endif             // --------------------------------------------------------------------- CSHARP

%}

%shared_ptr(chrono::ChLinkRSDA)
%shared_ptr(chrono::ChLinkRSDA::TorqueFunctor)

#ifdef SWIGCSHARP
%feature("director") ChTorqueFunctorP;
#endif

#ifdef SWIGPYTHON
%feature("director") TorqueFunctor;
#endif
 
// Tell SWIG about parent class
%import "ChLinkMarkers.i"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES

class ChTorqueFunctorP {
    public:
        virtual ~ChTorqueFunctorP() {}
        virtual double evaluate(double time,
                                double angle,
                                double vel,
                                chrono::ChLinkRSDA* link) {
            return 0.0;
        }
};

%extend chrono::ChLinkRSDA
{
    void RegisterTorqueFunctor(std::shared_ptr<::ChTorqueFunctorP> functor) {
       $self->RegisterTorqueFunctor(functor);
    }
}

%ignore chrono::ChLinkRSDA::RegisterTorqueFunctor();

#endif             // --------------------------------------------------------------------- CSHARP

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChLinkRSDA.h"
