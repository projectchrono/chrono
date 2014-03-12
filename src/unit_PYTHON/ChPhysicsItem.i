%{

/* Includes the header in the wrapper code */
#include "physics/ChPhysicsItem.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// This is needed because a std::vector< ChSharedPtr<ChAsset> > 
// is used as a type in this class, and we want to access std via python
%template(vector_ChAsset) std::vector< chrono::ChSharedPtr<chrono::ChAsset> >;


/* Parse the header file to generate wrappers */
%include "../physics/ChPhysicsItem.h"    

// Define also the shared pointer chrono::ChShared<ChAsset> 
// (renamed as 'ChAssetShared' in python)

%DefChSharedPtr(chrono::,ChPhysicsItem)