#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChLinkTSDA::GetFrame1Rel "public override"
%csmethodmodifiers chrono::ChLinkTSDA::GetFrame2Rel "public override"

#endif             // --------------------------------------------------------------------- CSHARP



%{
#include "chrono/physics/ChLinkTSDA.h"

using namespace chrono;

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class TSDAForceFunctor : public chrono::ChLinkTSDA::ForceFunctor {
    public:
        TSDAForceFunctor() {}
        virtual double evaluate(double time,
                                double rest_length,
                                double length,
                                double vel,
                                const chrono::ChLinkTSDA& link) override {
            std::cout << "You must implement the function evaluate()!" << std::endl;
            return 0.0;
        }
};

#endif             // --------------------------------------------------------------------- CSHARP

%}

%shared_ptr(chrono::ChLinkTSDA)
%shared_ptr(chrono::ChLinkTSDA::ForceFunctor)

#ifdef SWIGCSHARP
%feature("director") TSDAForceFunctor;
#endif

#ifdef SWIGPYTHON
%feature("director") ForceFunctor;
#endif
 
// Tell SWIG about parent class in Python
%import "ChLink.i"

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// NESTED CLASSES

class TSDAForceFunctor {
    public:
        virtual ~TSDAForceFunctor() {}
        virtual double evaluate(double time,
                                double rest_length,
                                double length,
                                double vel,
                                const chrono::ChLinkTSDA& link) {
            return 0.0;
        }
};

%extend chrono::ChLinkTSDA
{
    void RegisterForceFunctor(std::shared_ptr<::TSDAForceFunctor> functor) {
       $self->RegisterForceFunctor(functor);
    }
}

%ignore chrono::ChLinkTSDA::RegisterForceFunctor();

#endif             // --------------------------------------------------------------------- CSHARP

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChLinkTSDA.h"
