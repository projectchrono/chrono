%{

/* Includes the header in the wrapper code */
#include <cstdlib>
#include <cmath>
#include <cstdlib>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChIntegrable.h"

using namespace chrono;
%}

// Tell SWIG about parent class in Python
%import "ChVectorDynamic.i"

/* Parse the header file to generate wrappers */

%shared_ptr(chrono::ChIntegrable)
%shared_ptr(chrono::ChIntegrableIIorder)
  
%include "../chrono/timestepper/ChState.h"
%include "../chrono/timestepper/ChIntegrable.h"
%include "../chrono/timestepper/ChTimestepper.h"
%include "../chrono/timestepper/ChTimestepperHHT.h"






