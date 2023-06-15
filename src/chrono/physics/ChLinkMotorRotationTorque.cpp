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

#include "chrono/physics/ChLinkMotorRotationTorque.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationTorque)

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque() {
    this->c_rz = false;
    SetupLinkMask();

    m_func = chrono_types::make_shared<ChFunction_Const>(0.0);
}

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other)
    : ChLinkMotorRotation(other) {}

ChLinkMotorRotationTorque::~ChLinkMotorRotationTorque() {}

void ChLinkMotorRotationTorque::Update(double mytime, bool update_assets) {
    ChLinkMotorRotation::Update(mytime, update_assets);
}

void ChLinkMotorRotationTorque::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant torque
    double mT = m_func->Get_y(this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA() * ChVector<>(0, 0, mT);

    if (Body2->Variables().IsActive()) {
        R.segment(Body2->Variables().GetOffset() + 3, 3) -=
            c * Body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
    }

    if (Body1->Variables().IsActive()) {
        R.segment(Body1->Variables().GetOffset() + 3, 3) +=
            c * Body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
    }
}

void ChLinkMotorRotationTorque::ConstraintsFbLoadForces(double factor) {
    // compute instant torque
    double mT = m_func->Get_y(this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA() * ChVector<>(0, 0, mT);

    Body2->Variables().Get_fb().segment(3, 3) -= factor * Body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
    Body1->Variables().Get_fb().segment(3, 3) += factor * Body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
}

void ChLinkMotorRotationTorque::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationTorque>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationTorque::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorRotationTorque>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
