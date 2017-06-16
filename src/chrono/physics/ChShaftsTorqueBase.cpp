// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorqueBase.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChShaftsTorqueBase)  // NO! Abstract class

ChShaftsTorqueBase::ChShaftsTorqueBase() : torque(0) {}

ChShaftsTorqueBase::ChShaftsTorqueBase(const ChShaftsTorqueBase& other) : ChShaftsCouple(other) {
    torque = other.torque;
}

void ChShaftsTorqueBase::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    torque = ComputeTorque();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueBase::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                           ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                           const double c           // a scaling factor
                                           ) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += torque * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += -torque * c;
}

// SOLVER INTERFACES

void ChShaftsTorqueBase::VariablesFbLoadForces(double factor) {
    // add applied torques to 'fb' vector
    shaft1->Variables().Get_fb().ElementN(0) += torque * factor;
    shaft2->Variables().Get_fb().ElementN(0) += -torque * factor;
}

//////// FILE I/O

void ChShaftsTorqueBase::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsTorqueBase>();

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(torque);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorqueBase::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsTorqueBase>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(torque);
}

}  // end namespace chrono
