%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChCollisionModel.h"

using namespace collision;

%}

%feature("director") chrono::collision::ChCollisionModel; // ?????
%feature("nodirector") chrono::collision::ChCollisionModel::GetPhysicsItem;

/* Parse the header file to generate wrappers */
%include "../../chrono/collision/ChCollisionModel.h"

