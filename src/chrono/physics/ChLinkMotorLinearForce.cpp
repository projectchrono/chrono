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

#include "chrono/physics/ChLinkMotorLinearForce.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearForce)

ChLinkMotorLinearForce::ChLinkMotorLinearForce() {
    this->c_z = false;  // no actual constraint is set on the actuated axis
    SetupLinkMask();
    m_func = chrono_types::make_shared<ChFunctionConst>(0.0);
}

ChLinkMotorLinearForce::ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other) : ChLinkMotorLinear(other) {}

ChLinkMotorLinearForce::~ChLinkMotorLinearForce() {}

void ChLinkMotorLinearForce::Update(double time, bool update_assets) {
    ChLinkMotorLinear::Update(time, update_assets);
}

void ChLinkMotorLinearForce::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant force
    double mF = m_func->GetVal(this->GetChTime());

    ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
    ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);

    // application point is always marker1
    ChVector3d m_abs_force = aframe2.GetRotMat() * ChVector3d(0, 0, mF);

    if (m_body2->Variables().IsActive()) {
        auto w2_abs = m_body2->AppliedForceParentToWrenchParent(m_abs_force, aframe1.GetPos());
        R.segment(m_body2->Variables().GetOffset() + 0, 3) -= c * w2_abs.force.eigen();
        R.segment(m_body2->Variables().GetOffset() + 3, 3) -=
            c * m_body2->TransformDirectionParentToLocal(w2_abs.torque).eigen();
    }

    if (m_body1->Variables().IsActive()) {
        auto w1_abs = m_body1->AppliedForceParentToWrenchParent(m_abs_force, aframe1.GetPos());
        R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * w1_abs.force.eigen();
        R.segment(m_body1->Variables().GetOffset() + 3, 3) +=
            c * m_body1->TransformDirectionParentToLocal(w1_abs.torque).eigen();
    }
}

void ChLinkMotorLinearForce::ConstraintsFbLoadForces(double factor) {
    // compute instant force
    double mF = m_func->GetVal(this->GetChTime());

    ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
    ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);

    // application point is always marker1
    ChVector3d m_abs_force = aframe2.GetRotMat() * ChVector3d(0, 0, mF);

    auto w2_abs = m_body2->AppliedForceParentToWrenchParent(m_abs_force, aframe1.GetPos());
    m_body2->Variables().Force().segment(0, 3) -= factor * w2_abs.force.eigen();
    m_body2->Variables().Force().segment(3, 3) -=
        factor * m_body2->TransformDirectionParentToLocal(w2_abs.torque).eigen();

    auto w1_abs = m_body1->AppliedForceParentToWrenchParent(m_abs_force, aframe1.GetPos());
    m_body1->Variables().Force().segment(0, 3) += factor * w1_abs.force.eigen();
    m_body1->Variables().Force().segment(3, 3) +=
        factor * m_body1->TransformDirectionParentToLocal(w1_abs.torque).eigen();
}

void ChLinkMotorLinearForce::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorLinearForce>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOut(archive_out);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearForce::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorLinearForce>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIn(archive_in);

    // deserialize all member data:
}

}  // end namespace chrono
