#ifndef CHC_NARROWPHASE_GJK_EPA_H
#define CHC_NARROWPHASE_GJK_EPA_H

#include "chrono_parallel/collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {

struct sResults {
   enum eStatus {
      Separated, /* Shapes does not penetrate                                   */
      Penetrating, /* Shapes are penetrating                                  */
      GJK_Failed, /* GJK phase fail, no big issue, shapes are probably just 'touching' */
      EPA_Failed /* EPA phase fail, bigger problem, need to save parameters, and debug */
   } status;
   real3 witnesses[2];
   real3 normal;
   real distance;
};

CH_PARALLEL_API
bool GJKDistance(const ConvexShape& shape0,
                 const ConvexShape& shape1,
                 const real3& guess,
                 sResults& results);
CH_PARALLEL_API
bool GJKPenetration(const ConvexShape& shape0,
                    const ConvexShape& shape1,
                    const real3& guess,
                    sResults& results);

CH_PARALLEL_API
bool GJKCollide(const ConvexShape& shape0,
                    const ConvexShape& shape1,
                    sResults& results);

CH_PARALLEL_API
bool GJKFindPenetration(const ConvexShape& shape0,
                const ConvexShape& shape1,
                sResults& results);


}  // end namespace collision
}  // end namespace chrono

#endif

