%{

/* Includes the header in the wrapper code */
#include "chrono/collision/ChCCollisionModel.h"

using namespace collision;

%}


%feature("director") chrono::collision::ChCollisionModel; // ?????


/* Parse the header file to generate wrappers */
%include "../chrono/collision/ChCCollisionModel.h"

