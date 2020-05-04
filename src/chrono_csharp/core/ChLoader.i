%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoader.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoaderUVW.h"


%}

//TODO: remove ignore once ref is wrapped 
%shared_ptr(chrono::ChLoader)
%shared_ptr(chrono::ChLoaderU)
%shared_ptr(chrono::ChLoaderUdistributed)
%shared_ptr(chrono::ChLoaderUatomic)
%shared_ptr(chrono::ChLoaderUV)
%shared_ptr(chrono::ChLoaderUVdistributed)
%shared_ptr(chrono::ChLoaderUVatomic)
%shared_ptr(chrono::ChLoaderForceOnSurface)
%shared_ptr(chrono::ChLoaderPressure)
%shared_ptr(chrono::ChLoaderUVW)
%shared_ptr(chrono::ChLoaderUVWdistributed)
%shared_ptr(chrono::ChLoaderUVWatomic)
%shared_ptr(chrono::ChLoaderGravity)
%shared_ptr(chrono::ChLoaderXYZnode)



// Tell SWIG about parent class in Python
//%import "ChPhysicsItem.i"
//%import "ChObject.i"

/* Parse the header file to generate wrappers */

%include "../../chrono/physics/ChLoader.h"
%include "../../chrono/physics/ChLoaderU.h"
%include "../../chrono/physics/ChLoaderUV.h"
%include "../../chrono/physics/ChLoaderUVW.h"