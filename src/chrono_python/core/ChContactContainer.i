%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactable.h"

//using namespace collision;


// NESTED CLASSES: inherit stubs (not virtual) as outside classes

/*class ChReportContactCallbackP : public chrono::ChContactContainer::ReportContactCallback {
    public:
        ChReportContactCallbackP() {}
        virtual bool OnReportContact(const chrono::ChVector<>& pA,
                                     const chrono::ChVector<>& pB,
                                     const chrono::ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const chrono::ChVector<>& react_forces,
                                     const chrono::ChVector<>& react_torques,
                                     chrono::ChContactable* contactobjA,
                                     chrono::ChContactable* contactobjB) {
            GetLog() << "You must implement OnReportContact()!\n";
            return false;
        }
};

class ChAddContactCallbackP : public chrono::ChContactContainer::AddContactCallback {
    public:
        ChAddContactCallbackP() {}
        virtual void OnAddContact(const chrono::collision::ChCollisionInfo& contactinfo,
                                  chrono::ChMaterialComposite* const material) {
            GetLog() << "You must implement OnAddContact()!\n";
        }
};*/

%}

%shared_ptr(chrono::ChContactContainer::ReportContactCallback)
%shared_ptr(chrono::ChContactContainer::AddContactCallback)

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


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.

%feature("director") ChReportContactCallbackP;
%feature("director") ChAddContactCallbackP;
%feature("director") ReportContactCallback;


// NESTED CLASSES

/*class ChReportContactCallbackP {
  public:
    virtual ~ChReportContactCallbackP() {}
    virtual bool OnReportContact(const chrono::ChVector<>& pA,
                                 const chrono::ChVector<>& pB,
                                 const chrono::ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const chrono::ChVector<>& react_forces,
                                 const chrono::ChVector<>& react_torques,
                                 chrono::ChContactable* contactobjA,
                                 chrono::ChContactable* contactobjB) {
        return false;
    }
};

class ChAddContactCallbackP {
  public:
    virtual ~ChAddContactCallbackP() {}
    virtual void OnAddContact(const chrono::collision::ChCollisionInfo& contactinfo,
                              chrono::ChMaterialComposite* const material) {}
};

%extend chrono::ChContactContainer
{
	void RegisterAddContactCallback(::ChAddContactCallbackP* callback) {
	    $self->RegisterAddContactCallback(callback);
	}

	void ReportAllContacts(::ChReportContactCallbackP* callback) {
	     $self->ReportAllContacts(callback);
	}
};
*/
//%ignore chrono::ChContactContainer::RegisterAddContactCallback();
//%ignore chrono::ChContactContainer::ReportAllContacts();


/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChContactable.h"
%include "../../chrono/physics/ChContactContainer.h"    

