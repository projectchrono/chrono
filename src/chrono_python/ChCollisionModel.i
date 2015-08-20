%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionModel.h"

using namespace collision;

%}


// Forward ref
//%import "ChXxxxx.i"
//%import "ChVector.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


%feature("director") chrono::collision::ChCollisionModel; // ?????


// This is needed because a std::vector<ChVector<double>
// is used as a type in this class, and we want to access std via python
%template(vector_ChVectorD) std::vector< chrono::ChVector<double> >;

/* Parse the header file to generate wrappers */
%include "../collision/ChCCollisionModel.h"

