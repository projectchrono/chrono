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

class CH_PARALLEL_API ChCNarrowphase {
 public:
   ChCNarrowphase()
         : total_possible_contacts(0) {
   }
   virtual ~ChCNarrowphase() {
   }

   virtual void Process(ChParallelDataManager* data_container) = 0;
   virtual void Update(ChParallelDataManager* data_container) = 0;

   void SetCollisionEnvelope(const real &envelope) {
      collision_envelope = envelope;
   }
   real GetCollisionEnvelope() {
      return collision_envelope;
   }

 protected:
   uint total_possible_contacts;
   real collision_envelope;
};

}  // end namespace collision
}  // end namespace chrono

#endif

