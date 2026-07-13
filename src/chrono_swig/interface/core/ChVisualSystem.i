%{
#include "chrono/assets/ChVisualSystem.h"
using namespace chrono;
%}

namespace chrono {
class ChPhysicsItem;
}

%shared_ptr(chrono::ChVisualSystem)

%rename(VisualSettings) chrono::ChVisualSystem::Settings;

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChVisualSystem.h"    
