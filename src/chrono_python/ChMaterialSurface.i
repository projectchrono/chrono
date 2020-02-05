%{

/* Includes the header in the wrapper code */

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
using namespace chrono;

%};



/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChMaterialSurface.h"  
%include "../chrono/physics/ChMaterialSurfaceNSC.h"  
%include "../chrono/physics/ChMaterialSurfaceSMC.h"  


