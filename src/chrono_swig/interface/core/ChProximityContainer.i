%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChProximityContainer.h"


// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class ChReportProximityCallbackP : public chrono::ChProximityContainer::ReportProximityCallback {
    public:
        ChReportProximityCallbackP() {}
        virtual bool OnReportProximity(chrono::ChCollisionModel* modA,
                                       chrono::ChCollisionModel* modB) {
            GetLog() << "You must implement OnReportProximity()!\n";
            return false;
        }
};

class ChAddProximityCallbackP : public chrono::ChProximityContainer::AddProximityCallback {
    public:
        ChAddProximityCallbackP() {}
        virtual void OnAddProximity(const chrono::ChCollisionModel& modA,
                                    const chrono::ChCollisionModel& modB) {
            GetLog() << "You must implement OnAddProximity()!\n";
        }
};

%}


// Forward ref
%import "ChCollisionModel.i"


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") ChAddProximityCallbackP;
%feature("director") ChReportProximityCallbackP;


// NESTED CLASSES

class ChReportProximityCallbackP {
  public:
    virtual ~ChReportProximityCallbackP() {}
    virtual bool OnReportProximity(chrono::ChCollisionModel* modA,
                                   chrono::ChCollisionModel* modB) {
        return false;
    }
};

class ChAddProximityCallbackP {
  public:
    virtual ~ChAddProximityCallbackP() {}
    virtual void OnAddProximity(const chrono::ChCollisionModel& modA,
                                const chrono::ChCollisionModel& modB) {}
};

%extend chrono::ChProximityContainer
{
    void RegisterAddProximityCallback(::ChAddProximityCallbackP* callback) {
        $self->RegisterAddProximityCallback(callback);
    }

    void ReportAllProximities(::ChReportProximityCallbackP* callback) {
        $self->ReportAllProximities(callback);
    }
};

%ignore chrono::ChProximityContainer::RegisterAddProximityCallback();
%ignore chrono::ChProximityContainer::ReportAllProximities();


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChProximityContainer.h"    

