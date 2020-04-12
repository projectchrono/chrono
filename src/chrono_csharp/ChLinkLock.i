%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLinkLock.h"

%}
 

%shared_ptr(chrono::ChLinkLock)
%shared_ptr(chrono::ChLinkLockRevolute)
%shared_ptr(chrono::ChLinkLockLock)
%shared_ptr(chrono::ChLinkLockSpherical)
%shared_ptr(chrono::ChLinkLockCylindrical)
%shared_ptr(chrono::ChLinkLockPrismatic)
%shared_ptr(chrono::ChLinkLockPointPlane)
%shared_ptr(chrono::ChLinkLockPointLine)
%shared_ptr(chrono::ChLinkLockPlanePlane)
%shared_ptr(chrono::ChLinkLockOldham)
%shared_ptr(chrono::ChLinkLockFree)
%shared_ptr(chrono::ChLinkLockAlign)
%shared_ptr(chrono::ChLinkLockParallel)
%shared_ptr(chrono::ChLinkLockPerpend)
%shared_ptr(chrono::ChLinkLockRevolutePrismatic)

// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkForce.h"
%include "../chrono/physics/ChLinkLock.h"  
