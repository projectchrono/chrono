///////////////////////////////////////////////////
//
//   ChContactContainerGPU.cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChContactContainerGPU.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletParticle.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono {
    using namespace collision;
    using namespace geometry;

    // Register into the object factory, to enable run-time
    // dynamic creation and persistence
    ChClassRegister<ChContactContainerGPU> a_registration_ChContactContainerGPU;

    ChContactContainerGPU::ChContactContainerGPU() {
        n_added = 0;
    }

    ChContactContainerGPU::~ChContactContainerGPU() {
        n_added = 0;
    }

} // END_OF_NAMESPACE____

