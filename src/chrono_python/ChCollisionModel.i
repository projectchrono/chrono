%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChCCollisionModel.h"

using namespace collision;

%}


//%feature("director") chrono::collision::ChCollisionModel; // ?????

//%ignore chrono::ChCollisionModel::GetPhysicsItem();
/* Parse the header file to generate wrappers */
%include "../chrono/collision/ChCCollisionModel.h"

