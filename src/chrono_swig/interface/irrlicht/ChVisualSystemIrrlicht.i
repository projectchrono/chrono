%{

/* Includes the header in the wrapper code */
#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../../../chrono_irrlicht/ChIrrNodeShape.h"    
%include "../../../chrono_irrlicht/ChIrrNodeModel.h"    
%include "../../../chrono_irrlicht/ChVisualSystemIrrlicht.h"
