%{
#include "chrono/collision/ChCollisionModel.h"

using namespace chrono;
%}



#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

%feature("director") chrono::ChCollisionModel; // ?????
%feature("nodirector") chrono::ChCollisionModel::GetPhysicsItem;

#endif             // --------------------------------------------------------------------- PYTHON


%ignore chrono::ChCollisionModelBullet;
%ignore chrono::ChCollisionModelMulticore;

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionModel.h"
