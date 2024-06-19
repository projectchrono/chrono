%{
#ifdef CHRONO_COLLISION
/* Includes the headers in the wrapper code */
#include "chrono/multicore_math/matrix.h"
#include "chrono/multicore_math/other_types.h"
#include "chrono/multicore_math/real.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"
#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/utility.h"

#include "chrono/collision/chrono/ChCollisionModelChrono.h"
#include "chrono/collision/chrono/ChCollisionSystemChrono.h"
#include "chrono/collision/chrono/ChBroadphase.h"
#include "chrono/collision/chrono/ChCollisionData.h"
#include "chrono/collision/chrono/ChCollisionUtils.h"
#include "chrono/collision/chrono/ChConvexShape.h"
#include "chrono/collision/chrono/ChNarrowphase.h"
#include "chrono/collision/chrono/ChRayTest.h"


using namespace ch_utils;
#endif
%}


/* Import ChSystem to make sure CHRONO_COLLISION is defined! */
%import "ChSystem.i"

#ifdef CHRONO_COLLISION

/* Enable shared pointers */
%shared_ptr(chrono::ChCollisionModelChrono)

/* Parse the header files to generate wrappers */
%include "../../../chrono/multicore_math/matrix.h"
%include "../../../chrono/multicore_math/other_types.h"
%include "../../../chrono/multicore_math/real.h"
%include "../../../chrono/multicore_math/real2.h"
%include "../../../chrono/multicore_math/real3.h"
%include "../../../chrono/multicore_math/real4.h"
%include "../../../chrono/multicore_math/simd.h"
%include "../../../chrono/multicore_math/utility.h"

%include "../../../chrono/collision/chrono/ChCollisionModelChrono.h"
%include "../../../chrono/collision/chrono/ChCollisionSystemChrono.h"
%include "../../../chrono/collision/chrono/ChBroadphase.h"
%include "../../../chrono/collision/chrono/ChCollisionData.h"
%include "../../../chrono/collision/chrono/ChCollisionUtils.h"
%include "../../../chrono/collision/chrono/ChConvexShape.h"
%include "../../../chrono/collision/chrono/ChNarrowphase.h"
%include "../../../chrono/collision/chrono/ChRayTest.h"

#endif
