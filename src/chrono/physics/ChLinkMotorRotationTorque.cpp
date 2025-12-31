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

    m_func = chrono_types::make_shared<ChFunctionConst>(0.0);
}

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other)
    : ChLinkMotorRotation(other) {}

ChLinkMotorRotationTorque::~ChLinkMotorRotationTorque() {}

void ChLinkMotorRotationTorque::Update(double time, bool update_assets) {
    ChLinkMotorRotation::Update(time, update_assets);
}

void ChLinkMotorRotationTorque::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant torque
    double mT = m_func->GetVal(this->GetChTime());

    ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
    ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);
    ChVector3d m_abs_torque = aframe2.GetRotMat() * ChVector3d(0, 0, mT);

    if (m_body2->Variables().IsActive()) {
        R.segment(m_body2->Variables().GetOffset() + 3, 3) -=
            c * m_body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
    }

    if (m_body1->Variables().IsActive()) {
        R.segment(m_body1->Variables().GetOffset() + 3, 3) +=
            c * m_body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
    }
}

void ChLinkMotorRotationTorque::ConstraintsFbLoadForces(double factor) {
    // compute instant torque
    double mT = m_func->GetVal(this->GetChTime());

    ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
    ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);
    ChVector3d m_abs_torque = aframe2.GetRotMat() * ChVector3d(0, 0, mT);

    m_body2->Variables().Force().segment(3, 3) -=
        factor * m_body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
    m_body1->Variables().Force().segment(3, 3) +=
        factor * m_body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
}

void ChLinkMotorRotationTorque::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorRotationTorque>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(archive_out);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationTorque::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorRotationTorque>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(archive_in);

    // deserialize all member data:
}

}  // end namespace chrono
