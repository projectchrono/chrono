#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChLoadBodyInertia::LoadIntLoadResidual_Mv "public override"

#endif             // --------------------------------------------------------------------- CSHARP


%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsXYZnode.h"

%}

//TODO: remove ignore once ref is wrapped 
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoad)
%shared_ptr(chrono::ChLoadCustom)
%shared_ptr(chrono::ChLoadCustomMultiple)
%shared_ptr(chrono::ChLoadBodyForce)
%shared_ptr(chrono::ChLoadBodyTorque)
%shared_ptr(chrono::ChLoadBodyInertia)
%shared_ptr(chrono::ChLoadBodyBody)
%shared_ptr(chrono::ChLoadBodyBodyTorque)
%shared_ptr(chrono::ChLoadBodyBodyBushingSpherical)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingMate)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingGeneric)
%shared_ptr(chrono::ChLoadXYZnode)
%shared_ptr(chrono::ChLoadXYZnodeForce)
%shared_ptr(chrono::ChLoadXYZnodeForceAbsolute)
%shared_ptr(chrono::ChLoadXYZnodeXYZnode)
%shared_ptr(chrono::ChLoadXYZnodeXYZnodeSpring)
%shared_ptr(chrono::ChLoadXYZnodeXYZnodeBushing)
%shared_ptr(chrono::ChLoadXYZnodeBody)
%shared_ptr(chrono::ChLoadXYZnodeBodySpring)
%shared_ptr(chrono::ChLoadXYZnodeBodyBushing)

// Tell SWIG about parent class in Python
%import "chrono_swig/interface/core/ChPhysicsItem.i"
%import "chrono_swig/interface/core/ChObject.i"

/* Parse the header file to generate wrappers */
// The user might inherit and construct ChLoadCustom
%feature("director") chrono::ChLoadBase;
%feature("director") chrono::ChLoad;
%feature("director") chrono::ChLoadCustom;
%feature("director") chrono::ChLoadCustomMultiple;

%template(vector_ChLoadable) std::vector< std::shared_ptr< chrono::ChLoadable >>;

%ignore chrono::ChLoadBase::ComputeJacobian;
%ignore chrono::ChLoadCustom::ComputeJacobian;
%ignore chrono::ChLoadCustom::Clone;
%ignore chrono::ChLoadCustomMultiple::ComputeJacobian;

%include "../../../chrono/physics/ChLoad.h"
%include "../../../chrono/physics/ChLoadsBody.h"
%include "../../../chrono/physics/ChLoadsXYZnode.h"
