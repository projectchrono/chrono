%{
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"

using namespace chrono;
%}



#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

%feature("director") chrono::ChCollisionModel; // ?????
%feature("nodirector") chrono::ChCollisionModel::GetPhysicsItem;

#endif             // --------------------------------------------------------------------- PYTHON

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionModel.h"
%include "../../../chrono/collision/bullet/ChCollisionModelBullet.h"