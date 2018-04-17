%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChAssembly.h"

using namespace chrono;

%}

%template(vector_ChBody) std::vector< std::shared_ptr<chrono::ChBody> >;
%template(vector_ChLink) std::vector< std::shared_ptr<chrono::ChLink> >;
%template(vector_ChPhysicsItem) std::vector< std::shared_ptr<chrono::ChPhysicsItem> >;


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChAssembly.h"    


