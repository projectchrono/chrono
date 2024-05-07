#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChLinkRSDA::GetFrame1Rel "public override"
%csmethodmodifiers chrono::ChLinkRSDA::GetFrame2Rel "public override"

#endif             // --------------------------------------------------------------------- CSHARP


%{
#include "chrono/physics/ChLinkRSDA.h"

using namespace chrono;

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class RSDATorqueFunctor : public chrono::ChLinkRSDA::TorqueFunctor {
    public:
        RSDATorqueFunctor() {}
        virtual double evaluate(double time,
                                double rest_angle,
                                double angle,
                                double vel,
                                const chrono::ChLinkRSDA& link) override {
            std::cout << "You must implement the function evaluate()!" << std::endl;
            return 0.0;
        }
};

#endif             // --------------------------------------------------------------------- CSHARP

%}

%shared_ptr(chrono::ChLinkRSDA)
%shared_ptr(chrono::ChLinkRSDA::TorqueFunctor)

#ifdef SWIGCSHARP
%feature("director") RSDATorqueFunctor;
#endif

#ifdef SWIGPYTHON
%feature("director") TorqueFunctor;
#endif
 
// Tell SWIG about parent class
%import "ChLinkMarkers.i"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES

class RSDATorqueFunctor {
    public:
        virtual ~RSDATorqueFunctor() {}
        virtual double evaluate(double time,
                                double rest_angle,
                                double angle,
                                double vel,
                                const chrono::ChLinkRSDA& link) {
            return 0.0;
        }
};

%extend chrono::ChLinkRSDA
{
    void RegisterTorqueFunctor(std::shared_ptr<::RSDATorqueFunctor> functor) {
       $self->RegisterTorqueFunctor(functor);
    }
}

%ignore chrono::ChLinkRSDA::RegisterTorqueFunctor();

#endif             // --------------------------------------------------------------------- CSHARP

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChLinkRSDA.h"
