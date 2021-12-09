%{
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionModelBullet.h"

using namespace collision;
%}

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

%feature("director") chrono::collision::ChCollisionModel; // ?????
%feature("nodirector") chrono::collision::ChCollisionModel::GetPhysicsItem;

#endif             // --------------------------------------------------------------------- PYTHON

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionModel.h"
%include "../../../chrono/collision/ChCollisionModelBullet.h"
