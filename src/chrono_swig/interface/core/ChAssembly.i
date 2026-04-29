%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChAssembly.h"

using namespace chrono;
#ifdef CHRONO_FEA
using namespace chrono::fea;
#endif

%}

%shared_ptr(chrono::ChAssembly)

%template(vector_ChBody) std::vector< std::shared_ptr<chrono::ChBody>>;
%template(vector_ChLink) std::vector< std::shared_ptr<chrono::ChLink>>;
%template(vector_ChLinkBase) std::vector< std::shared_ptr<chrono::ChLinkBase>>;
#ifdef CHRONO_FEA
%template(vector_ChMesh) std::vector< std::shared_ptr<chrono::fea::ChMesh>>;
#endif
%template(vector_ChPhysicsItem) std::vector< std::shared_ptr<chrono::ChPhysicsItem>>;


/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChAssembly.h"    


