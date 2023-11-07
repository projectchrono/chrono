%{
#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"

%}

// Forward ref
//%import "ChCollisionModel.i"

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionInfo.h"
