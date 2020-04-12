%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChAssembly.h"

using namespace chrono;
using namespace chrono::fea;

%}

%shared_ptr(chrono::ChAssembly)

%template(vector_ChBody) std::vector< std::shared_ptr<chrono::ChBody> >;
%template(vector_ChLink) std::vector< std::shared_ptr<chrono::ChLink> >;
%template(vector_ChMesh) std::vector< std::shared_ptr<chrono::fea::ChMesh> >;
%template(vector_ChPhysicsItem) std::vector< std::shared_ptr<chrono::ChPhysicsItem> >;


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChAssembly.h"    


