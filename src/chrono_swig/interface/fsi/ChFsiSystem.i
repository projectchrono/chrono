%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/ChFsiSystem.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;

%}

#ifdef SWIGPYTHON
%threadallow chrono::fsi::ChFsiSystem::DoStepDynamics;
#endif

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/ChFsiSystem.h"    
