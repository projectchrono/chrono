%{
#include "chrono/physics/ChLinkTSDA.h"

using namespace chrono;

// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class ChForceFunctorP : public chrono::ChLinkTSDA::ForceFunctor {
    public:
        ChForceFunctorP() {}
        virtual double evaluate(double time,
                                double rest_length,
                                double length,
                                double vel,
                                chrono::ChLinkTSDA* link) override {
            GetLog() << "You must implement the function evaluate()!\n";
            return 0.0;
        }
};

%}

%feature("director") ChForceFunctorP;

%shared_ptr(chrono::ChLinkTSDA)
%shared_ptr(chrono::ChLinkTSDA::ForceFunctor)

// Tell SWIG about parent class
%import "ChLink.i"

// NESTED CLASSES

class ChForceFunctorP {
    public:
        virtual ~ChForceFunctorP() {}
        virtual double evaluate(double time,
                                double rest_length,
                                double length,
                                double vel,
                                chrono::ChLinkTSDA* link) {
            return 0.0;
        }
};

%extend chrono::ChLinkTSDA
{
    void RegisterForceFunctor(std::shared_ptr<::ChForceFunctorP> functor) {
       $self->RegisterForceFunctor(functor);
    }
}

%ignore chrono::ChLinkTSDA::RegisterForceFunctor();

%include "../../chrono/physics/ChLinkTSDA.h"
