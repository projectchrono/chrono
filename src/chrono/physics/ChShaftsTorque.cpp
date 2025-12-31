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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChShaftsTorque)  // NO! Abstract class

ChShaftsTorque::ChShaftsTorque() : torque(0) {}

ChShaftsTorque::ChShaftsTorque(const ChShaftsTorque& other) : ChShaftsCouple(other) {
    torque = other.torque;
}

void ChShaftsTorque::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(time, update_assets);

    // update class data
    torque = ComputeTorque();
}

void ChShaftsTorque::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                       ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                       const double c           // a scaling factor
) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += torque * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += -torque * c;
}

void ChShaftsTorque::VariablesFbLoadForces(double factor) {
    // add applied torques to 'fb' vector
    shaft1->Variables().Force()(0) += torque * factor;
    shaft2->Variables().Force()(0) += -torque * factor;
}

void ChShaftsTorque::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsTorque>();

    // serialize parent class
    ChShaftsCouple::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(torque);
}

void ChShaftsTorque::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsTorque>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(torque);
}

}  // end namespace chrono
