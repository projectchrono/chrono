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
        this->load_C = 0;
        this->load_max_recovery_speed = 0;
        this->load_do_clamp = true;
    }

    ChContactContainerGPU::~ChContactContainerGPU() {
        n_added = 0;
    }

    void ChContactContainerGPU::Update(double mytime) {
        // Inherit time changes of parent class, basically doing nothing :)
        ChContactContainerBase::Update(mytime);
    }

    void ChContactContainerGPU::RemoveAllContacts() {
        n_added = 0;
    }

    void ChContactContainerGPU::BeginAddContact() {
        n_added = 0;
    }

    void ChContactContainerGPU::EndAddContact() {
    }

    void ChContactContainerGPU::AddContact(const collision::ChCollisionInfo &mcontact) {
        // Fetch the frames of that contact and other infos
    }

    void ChContactContainerGPU::ReportAllContacts(ChReportContactCallback *mcallback) {
    }

    ////////// LCP INTERFACES ////

    void ChContactContainerGPU::InjectConstraints(ChLcpSystemDescriptor &mdescriptor) {
    }

    void ChContactContainerGPU::ConstraintsBiReset() {
    }

    void ChContactContainerGPU::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool mdo_clamp) {
        this->load_C = factor;
        this->load_max_recovery_speed = recovery_clamp;
        this->load_do_clamp = mdo_clamp;
    }

    void ChContactContainerGPU::ConstraintsLoadJacobians() {
        // already loaded when ChContactGPUsimple objects are created
    }

    void ChContactContainerGPU::ConstraintsFetch_react(double factor) {
        // From constraints to react vector:
    }

    // Following functions are for exploiting the contact persistence

    void  ChContactContainerGPU::ConstraintsLiLoadSuggestedSpeedSolution() {
        // Fetch the last computed impulsive reactions from the persistent contact manifold (could
        // be used for warm starting the CCP speed solver):
    }

    void  ChContactContainerGPU::ConstraintsLiLoadSuggestedPositionSolution() {
        // Fetch the last computed 'positional' reactions from the persistent contact manifold (could
        // be used for warm starting the CCP position stabilization solver):
    }

    void  ChContactContainerGPU::ConstraintsLiFetchSuggestedSpeedSolution() {
        // Store the last computed reactions into the persistent contact manifold (might
        // be used for warm starting CCP the speed solver):
    }

    void  ChContactContainerGPU::ConstraintsLiFetchSuggestedPositionSolution() {
        // Store the last computed 'positional' reactions into the persistent contact manifold (might
        // be used for warm starting the CCP position stabilization solver):
    }
} // END_OF_NAMESPACE____

