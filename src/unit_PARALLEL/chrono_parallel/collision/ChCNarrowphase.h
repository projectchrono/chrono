#ifndef CHC_NARROWPHASE_H
#define CHC_NARROWPHASE_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {

struct ConvexShape {
   shape_type type;  //type of shape
   real3 A;  //location
   real3 B;  //dimensions
   real3 C;  //extra
   quaternion R;  //rotation
};

}  // end namespace collision
}  // end namespace chrono

#endif

