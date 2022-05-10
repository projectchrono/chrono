%{
#include "chrono/assets/ChVisualModel.h"
using namespace chrono;
%}

namespace chrono {
class ChPhysicsItem;
}

%shared_ptr(chrono::ChVisualModel)
%shared_ptr(chrono::ChVisualModelInstance)

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChVisualModel.h"    
