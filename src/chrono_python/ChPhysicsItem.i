%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChPhysicsItem.h"

using namespace chrono;

%}
 

// This is needed because a std::vector< std::shared_ptr<ChAsset> > 
// is used as a type in this class, and we want to access std via python
%template(vector_ChAsset) std::vector< std::shared_ptr<chrono::ChAsset> >;

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChPhysicsItem.h"    

