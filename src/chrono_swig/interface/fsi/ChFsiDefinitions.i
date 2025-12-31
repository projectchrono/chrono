%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/ChFsiDefinitions.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;

%}

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/ChFsiDefinitions.h"    
