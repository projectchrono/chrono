///////////////////////////////////////////////////
//
//   ChContactContainerGPUsimple.cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChContactContainerGPUsimple.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletParticle.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{
    using namespace collision;
    using namespace geometry;

    // Register into the object factory, to enable run-time
    // dynamic creation and persistence
    ChClassRegister<ChContactContainerGPUsimple> a_registration_ChContactContainerGPUsimple;

    ChContactContainerGPUsimple::ChContactContainerGPUsimple()
    {
        n_added = 0;
        this->load_C = 0;
        this->load_max_recovery_speed = 0;
        this->load_do_clamp = true;
    }

    ChContactContainerGPUsimple::~ChContactContainerGPUsimple()
    {
        n_added = 0;
    }

    void ChContactContainerGPUsimple::Update(double mytime)
    {
        // Inherit time changes of parent class, basically doing nothing :)
        ChContactContainerBase::Update(mytime);
    }

    void ChContactContainerGPUsimple::RemoveAllContacts()
    {
        n_added = 0;
    }

    void ChContactContainerGPUsimple::BeginAddContact()
    {
        n_added = 0;
    }

    void ChContactContainerGPUsimple::EndAddContact()
    {
    }

    void ChContactContainerGPUsimple::AddContact(const collision::ChCollisionInfo &mcontact)
    {
        // Fetch the frames of that contact and other infos
    }

    void ChContactContainerGPUsimple::ReportAllContacts(ChReportContactCallback *mcallback)
    {
    }

    ////////// LCP INTERFACES ////

    void ChContactContainerGPUsimple::InjectConstraints(ChLcpSystemDescriptor &mdescriptor)
    {
    }

    void ChContactContainerGPUsimple::ConstraintsBiReset()
    {
    }

    void ChContactContainerGPUsimple::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool mdo_clamp)
    {
        this->load_C = factor;
        this->load_max_recovery_speed = recovery_clamp;
        this->load_do_clamp = mdo_clamp;
    }

    void ChContactContainerGPUsimple::ConstraintsLoadJacobians()
    {
        // already loaded when ChContactGPUsimple objects are created
    }

    void ChContactContainerGPUsimple::ConstraintsFetch_react(double factor)
    {
        // From constraints to react vector:
    }

    // Following functions are for exploiting the contact persistence

    void  ChContactContainerGPUsimple::ConstraintsLiLoadSuggestedSpeedSolution()
    {
        // Fetch the last computed impulsive reactions from the persistent contact manifold (could
        // be used for warm starting the CCP speed solver):
    }

    void  ChContactContainerGPUsimple::ConstraintsLiLoadSuggestedPositionSolution()
    {
        // Fetch the last computed 'positional' reactions from the persistent contact manifold (could
        // be used for warm starting the CCP position stabilization solver):
    }

    void  ChContactContainerGPUsimple::ConstraintsLiFetchSuggestedSpeedSolution()
    {
        // Store the last computed reactions into the persistent contact manifold (might
        // be used for warm starting CCP the speed solver):
    }

    void  ChContactContainerGPUsimple::ConstraintsLiFetchSuggestedPositionSolution()
    {
        // Store the last computed 'positional' reactions into the persistent contact manifold (might
        // be used for warm starting the CCP position stabilization solver):
    }
} // END_OF_NAMESPACE____
