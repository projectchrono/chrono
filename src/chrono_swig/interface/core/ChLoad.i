%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsXYZnode.h"

%}

//TODO: remove ignore once ref is wrapped 
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoad)

%shared_ptr(chrono::ChLoad<chrono::ChLoaderGravity>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderXYZnode>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderU>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUdistributed>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUatomic>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUV>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUVdistributed>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUVatomic>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderForceOnSurface>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderPressure>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUVW>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUVWdistributed>)
%shared_ptr(chrono::ChLoad<chrono::ChLoaderUVWatomic>)

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
%feature("director") chrono::ChLoadCustom;
%feature("director") chrono::ChLoadCustomMultiple;
%template(vector_ChLoadable) std::vector< std::shared_ptr< chrono::ChLoadable >>;
%ignore chrono::ChLoadBase::ComputeJacobian;
%ignore chrono::ChLoadCustom::ComputeJacobian;
%ignore chrono::ChLoadCustom::Clone;
%ignore chrono::ChLoadCustomMultiple::ComputeJacobian;
%include "../../../chrono/physics/ChLoad.h"

%template(LoadLoaderXYZnode) chrono::ChLoad< chrono::ChLoaderXYZnode>;
%template(LoadLoaderGravity) chrono::ChLoad< chrono::ChLoaderGravity>;
%template(LoadLoaderForceOnSurface) chrono::ChLoad< chrono::ChLoaderForceOnSurface>;
%template(LoadLoaderPressure) chrono::ChLoad< chrono::ChLoaderPressure>;

//%template(LoadLoaderU) chrono::ChLoad< chrono::ChLoaderU>;
//%template(LoadLoaderUdistributed) chrono::ChLoad< chrono::ChLoaderUdistributed>;
//%template(LoadLoaderUatomic) chrono::ChLoad< chrono::ChLoaderUatomic>;
//%template(LoadLoaderLoaderUV) chrono::ChLoad< chrono::ChLoaderUV>;
//%template(LoadLoaderUVdistributed) chrono::ChLoad< chrono::ChLoaderUVdistributed>;
//%template(LoadLoaderUVatomic) chrono::ChLoad< chrono::ChLoaderUVatomic>;
//%template(LoadLoaderUVW) chrono::ChLoad< chrono::ChLoaderUVW>;
//%template(LoadLoaderUVWdistributed) chrono::ChLoad< chrono::ChLoaderUVWdistributed>;
//%template(LoadLoaderUVWatomic) chrono::ChLoad< chrono::ChLoaderUVWatomic>;


%include "../../../chrono/physics/ChLoadsBody.h"
%include "../../../chrono/physics/ChLoadsXYZnode.h"


