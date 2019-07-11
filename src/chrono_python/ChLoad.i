%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
//#include "chrono/physics/ChLoadsXYZnode.h"

%}
 
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoad)
%shared_ptr(chrono::ChLoadCustom)
%shared_ptr(chrono::ChLoadCustomMultiple)
%shared_ptr(chrono::ChLoadBodyForce)
%shared_ptr(chrono::ChLoadBodyTorque)
%shared_ptr(chrono::ChLoadBodyBody)
%shared_ptr(chrono::ChLoadBodyBodyTorque)
%shared_ptr(chrono::ChLoadBodyBodyBushingSpherical)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingMate)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingGeneric)
%shared_ptr(chrono::ChLoadXYZnodeForce)
%shared_ptr(chrono::ChLoadXYZnodeForceAbsolute)
%shared_ptr(chrono::ChLoadXYZnodeXYZnode)
%shared_ptr(chrono::ChLoadXYZnodeXYZnodeSpring)
%shared_ptr(chrono::ChLoadXYZnodeXYZnodeBushing)
%shared_ptr(chrono::ChLoadXYZnodeBody)
%shared_ptr(chrono::ChLoadXYZnodeBodySpring)
%shared_ptr(chrono::ChLoadXYZnodeBodyBushing)

// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"
%import "ChObject.i"

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLoad.h"
%include "../chrono/physics/ChLoadsBody.h"
//%include "../chrono/physics/ChLoadsXYZnode.h"




