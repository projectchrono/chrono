%{

/* Includes the header in the wrapper code */
#include "collision/ChCCollisionModel.h"

using namespace collision;

%}


%feature("director") chrono::collision::ChCollisionModel; // ?????

// This is needed because a std::vector<ChVector<double>
// is used as a type in this class, and we want to access std via python
%template(vector_ChVectorD) std::vector< chrono::ChVector<double> >;

/* Parse the header file to generate wrappers */
%include "../chrono/collision/ChCCollisionModel.h"

