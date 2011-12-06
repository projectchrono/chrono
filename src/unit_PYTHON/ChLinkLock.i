%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkLock.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMasked.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


/* Parse the header file to generate wrappers */
%include "../physics/ChLinkLock.h"  



// Define also the shared pointer chrono::ChShared<ChXxxx> 
// (renamed as 'ChXxxxShared' in python)

%DefChSharedPtr(ChLinkLockShared, ChLinkLock)
%DefChSharedPtr(ChLinkLockRevoluteShared, ChLinkLockRevolute)
%DefChSharedPtr(ChLinkLockLockShared, ChLinkLockLock)
%DefChSharedPtr(ChLinkLockSphericalShared, ChLinkLockSpherical)
%DefChSharedPtr(ChLinkLockCylindricalShared, ChLinkLockCylindrical)
%DefChSharedPtr(ChLinkLockPrismaticShared, ChLinkLockPrismatic)
%DefChSharedPtr(ChLinkLockPointPlaneShared, ChLinkLockPointPlane)
%DefChSharedPtr(ChLinkLockPointLineShared, ChLinkLockPointLine)
%DefChSharedPtr(ChLinkLockPlanePlaneShared, ChLinkLockPointPlane)
%DefChSharedPtr(ChLinkLockOldhamShared, ChLinkLockOldham)
%DefChSharedPtr(ChLinkLockFreeShared, ChLinkLockFree)
%DefChSharedPtr(ChLinkLockHookShared, ChLinkLockHook)
%DefChSharedPtr(ChLinkLockAlignShared, ChLinkLockAlign)
%DefChSharedPtr(ChLinkLockParallelShared, ChLinkLockParallel)
%DefChSharedPtr(ChLinkLockPerpendShared, ChLinkLockPerpend)