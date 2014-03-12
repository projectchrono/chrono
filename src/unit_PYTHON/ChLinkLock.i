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

%DefChSharedPtr(chrono::,ChLinkLock)
%DefChSharedPtr(chrono::,ChLinkLockRevolute)
%DefChSharedPtr(chrono::,ChLinkLockLock)
%DefChSharedPtr(chrono::,ChLinkLockSpherical)
%DefChSharedPtr(chrono::,ChLinkLockCylindrical)
%DefChSharedPtr(chrono::,ChLinkLockPrismatic)
%DefChSharedPtr(chrono::,ChLinkLockPointPlane)
%DefChSharedPtr(chrono::,ChLinkLockPointLine)
%DefChSharedPtr(chrono::,ChLinkLockPlanePlane)
%DefChSharedPtr(chrono::,ChLinkLockOldham)
%DefChSharedPtr(chrono::,ChLinkLockFree)
%DefChSharedPtr(chrono::,ChLinkLockHook)
%DefChSharedPtr(chrono::,ChLinkLockAlign)
%DefChSharedPtr(chrono::,ChLinkLockParallel)
%DefChSharedPtr(chrono::,ChLinkLockPerpend)