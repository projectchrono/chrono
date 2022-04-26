%{
#include "chrono/assets/ChVisualSystem.h"
using namespace chrono;
%}

namespace chrono {
class ChPhysicsItem;
}

%shared_ptr(chrono::ChVisualSystem)

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChVisualSystem.h"    
