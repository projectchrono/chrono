// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora 
// =============================================================================

#include <cstdlib>
#include <algorithm>

#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChFeeder.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/physics/ChContactContainerNSC.h"

namespace chrono {

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFeeder)


ChFeeder::ChFeeder()  {
    v_x = 0;
    v_y = 0;
    v_z = 0;
    w_x = 0;
    w_y = 0;
    w_z = 0;
}

ChFeeder::ChFeeder(const ChFeeder& other) : ChPhysicsItem(other) {
    feeder  = other.feeder;
    reference = other.reference;
    v_x = other.v_x;
    v_y = other.v_y;
    v_z = other.v_z;
    w_x = other.w_x;
    w_y = other.w_y;
    w_z = other.w_z;
}

ChFeeder::~ChFeeder() {
    
}

//// STATE BOOKKEEPING FUNCTIONS


void ChFeeder::IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) {

    // This feeder class works only if the contact container is of NSC type because we are going to modify the Ct = dC/dt component of the
    // constraints; the SMC contacts do not have it.
    
    if (auto mcontainer = std::dynamic_pointer_cast<ChContactContainerNSC>(this->system->GetContactContainer())) {

        // A class for iterating all over the NSC contacts. We assume that these have been already initialized and that
        // their offset is already set via some bookkeeping.
        class MyContactCallback : public ChContactContainerNSC::ReportContactCallbackNSC {
        public:
            ChFeeder* myfeeder;
            ChVectorDynamic<>* Qc;
            double c;

            virtual ~MyContactCallback() {}

            /// Callback used to report contact points already added to the container.
            /// If it returns false, the contact scanning will be stopped.
            virtual bool OnReportContact(
                const ChVector<>& pA,             ///< contact pA
                const ChVector<>& pB,             ///< contact pB
                const ChMatrix33<>& plane_coord,  ///< contact plane coordsystem (A column 'X' is contact normal)
                const double& distance,           ///< contact distance
                const double& eff_radius,         ///< effective radius of curvature at contact
                const ChVector<>& react_forces,   ///< react.forces (if already computed). In coordsystem 'plane_coord'
                const ChVector<>& react_torques,  ///< react.torques, if rolling friction (if already computed).
                ChContactable* contactobjA,  ///< model A (note: some containers may not support it and could be nullptr)
                ChContactable* contactobjB,  ///< model B (note: some containers may not support it and could be nullptr)
                const int offset  ///< offset of the first constraint (the normal component) in the vector of lagrangian multipliers, if already book-keeped
            ) {
                // Modify the Ct term in Qc only if the contact is touching a feeder object:
                if ((myfeeder->GetFeederObject().get() == contactobjA) || (myfeeder->GetFeederObject().get() == contactobjB)) {

                    // position of contact respect to the vibration mode reference, defining the screw motion 
                    ChVector<> local_rad = myfeeder->reference.TransformParentToLocal(pB);
                    // speed of the contact point, on the feeder (can be +/-), given the six values of the eigenvector screw motion
                    ChVector<> local_pseudovel = ChVector<>(myfeeder->v_x, myfeeder->v_y, myfeeder->v_z) +
                        Vcross(ChVector<>(myfeeder->w_x, myfeeder->w_y, myfeeder->w_z), local_rad);
                    ChVector<> pseudovel = myfeeder->reference.TransformDirectionLocalToParent(local_pseudovel);
                    // speed in the reference of the contact
                    ChVector<> contact_pseudovel = plane_coord.transpose() * pseudovel;

                    // The -Ct term to impose in the constraint.
                    // Note on signs: the Ct term should be negative for imposing positive contact-relative velocity, because of sign conventions in chrono.
                    ChVector<> contact_Ct; 
                    
                    // Heuristic formula: compute the tangential imposed speed, assuming other object is hit/moved only in ascending direction, and 
                    // assuming the speed is constant in all ascending phase (whereas it would be sinusoidal in reality, but
                    // would need some info on which point of the sinusoid, on average, is hit, and this might require some 
                    // statistical data or some info like gravity acceleration and frequency, which are not needed in the simple formula below)

                    if (contact_pseudovel.x() >= 0)  
                        contact_Ct = -contact_pseudovel;
                    else
                        contact_Ct = contact_pseudovel;

                    // Final touch: set the normal component of Ct as zero, to avoid unnecessary "pop corn" effect, that
                    // should tend to zero anyway in high frequency feeders:
                    contact_Ct.x() = 0;

                    // Write the Ct term in the Qc vector, as required by IntLoadConstraint_Ct
                    (*Qc)(offset) = contact_Ct.x() * c; // Ct =dC/dt term, N normal direction (
                    (*Qc)(offset + 1) = contact_Ct.y() * c; // Ct =dC/dt term, Tu direction 
                    (*Qc)(offset + 2) = contact_Ct.z() * c; // Ct =dC/dt term, Tv direction 
                }
                return true;
            }
        };

        auto mcallback = chrono_types::make_shared<MyContactCallback>();
        mcallback->myfeeder = this;
        mcallback->Qc = &Qc;
        mcallback->c = c;

        mcontainer->ReportAllContactsNSC(mcallback);

    }
    
  

}



void ChFeeder::Update(double mytime, bool update_assets) {
    // inherit parent class function
    ChPhysicsItem::Update(mytime, update_assets);

    
}

// FILE I/O

void ChFeeder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFeeder>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(marchive);

    // serialize all member data:
    //marchive << CHNVP(feeder);
    marchive << CHNVP(reference);
    marchive << CHNVP(v_x);
    marchive << CHNVP(v_y);
    marchive << CHNVP(v_z);
    marchive << CHNVP(w_x);
    marchive << CHNVP(w_y);
    marchive << CHNVP(w_z);
}

/// Method to allow de serialization of transient data from archives.
void ChFeeder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFeeder>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(marchive);

    // stream in all member data:
    //marchive >> CHNVP(feeder);
    marchive >> CHNVP(reference);
    marchive >> CHNVP(v_x);
    marchive >> CHNVP(v_y);
    marchive >> CHNVP(v_z);
    marchive >> CHNVP(w_x);
    marchive >> CHNVP(w_y);
    marchive >> CHNVP(w_z);
}









}  // end namespace chrono
