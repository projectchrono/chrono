%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChProximityContainerBase.h"

using namespace collision;


// NESTED CLASSES: inherit stubs (not virtual) as outside classes

class ChReportProximityCallbackP : public chrono::ChProximityContainerBase::ReportProximityCallback {
    public:
        ChReportProximityCallbackP() {}
        virtual bool OnReportProximity(chrono::collision::ChCollisionModel* modA,
                                       chrono::collision::ChCollisionModel* modB) {
            GetLog() << "You must implement OnReportProximity()!\n";
            return false;
        }
};

class ChAddProximityCallbackP : public chrono::ChProximityContainerBase::AddProximityCallback {
    public:
        ChAddProximityCallbackP() {}
        virtual void OnAddProximity(const chrono::collision::ChCollisionModel& modA,
                                    const chrono::collision::ChCollisionModel& modB) {
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
	virtual bool OnReportProximity(chrono::collision::ChCollisionModel* modA,
                                   chrono::collision::ChCollisionModel* modB) {return false;}
};

class ChAddProximityCallbackP {
  public:
	virtual void OnAddProximity(const chrono::collision::ChCollisionModel& modA,
                                const chrono::collision::ChCollisionModel& modB) {}
};

%extend chrono::ChProximityContainerBase
{
    void RegisterAddProximityCallback(::ChAddProximityCallbackP* callback) {
        $self->RegisterAddProximityCallback(callback);
    }
};

%ignore chrono::ChProximityContainerBase::RegisterAddProximityCallback();


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChProximityContainerBase.h"    

