#ifndef CHC_NARROWPHASE_GJK_EPA_H
#define CHC_NARROWPHASE_GJK_EPA_H

#include "chrono_parallel/collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {
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

// Minkovski Portal Refinement narrowphase collision detection
class CH_PARALLEL_API ChCNarrowphaseGJK_EPA : public ChCNarrowphase {
 public:
   ChCNarrowphaseGJK_EPA() {
   }

   virtual void Process(ChParallelDataManager* data_container) {

   }

   virtual void Update(ChParallelDataManager* data_container) {

   }

 private:

};

}  // end namespace collision
}  // end namespace chrono

#endif

