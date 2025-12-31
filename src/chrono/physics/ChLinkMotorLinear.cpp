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

#include "chrono/physics/ChLinkMotorLinear.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLinkMotorLinear)  NO! ABSTRACT!

ChLinkMotorLinear::ChLinkMotorLinear() : m_actuated_idx(0) {
    this->SetGuideConstraint(GuideConstraint::PRISMATIC);

    // DEVELOPER NOTES: c_z flag should be set by derived classes according to the type of constraint
    //                  e.g. force constraints has c_z=false since no proper constraint should be added

    mpos = 0;
    mpos_dt = 0;
    mpos_dtdt = 0;
}

ChLinkMotorLinear::ChLinkMotorLinear(const ChLinkMotorLinear& other) : ChLinkMotor(other) {
    mpos = other.mpos;
    mpos_dt = other.mpos_dt;
    mpos_dtdt = other.mpos_dtdt;
}

ChLinkMotorLinear::~ChLinkMotorLinear() {}

void ChLinkMotorLinear::SetGuideConstraint(bool mc_x, bool mc_y, bool mc_rx, bool mc_ry, bool mc_rz) {
    this->c_x = mc_x;
    this->c_y = mc_y;
    this->c_rx = mc_rx;
    this->c_ry = mc_ry;
    this->c_rz = mc_rz;
    SetupLinkMask();

    m_actuated_idx = (int)c_x + (int)c_y;
}

void ChLinkMotorLinear::SetGuideConstraint(const GuideConstraint mconstraint) {
    if (mconstraint == GuideConstraint::FREE) {
        this->c_x = false;
        this->c_y = false;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::PRISMATIC) {
        this->c_x = true;
        this->c_y = true;
        this->c_rx = true;
        this->c_ry = true;
        this->c_rz = true;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::SPHERICAL) {
        this->c_x = true;
        this->c_y = true;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }

    m_actuated_idx = (int)c_x + (int)c_y;
}

void ChLinkMotorLinear::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotor::Update(time, update_assets);

    // compute aux data for future reference (istantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(m_frame1) >> (ChFrameMoving<>)(*this->m_body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(m_frame2) >> (ChFrameMoving<>)(*this->m_body2);
    ChFrameMoving<> aframe12 = aframe2.TransformParentToLocal(aframe1);

    //// RADU TODO: revisit this.
    //// This is incorrect for GuideConstraint::FREE
    //// Should use something like sqrt(Vdot(relpos,relpos)), but taking into account sign?

    this->mpos = aframe12.GetPos().z();
    this->mpos_dt = aframe12.GetPosDt().z();
    this->mpos_dtdt = aframe12.GetPosDt2().z();
}

std::string ChLinkMotorLinear::GetGuideTypeString(GuideConstraint type) {
    switch (type) {
        case GuideConstraint::FREE:
            return "free";
        case GuideConstraint::PRISMATIC:
            return "prismatic";
        case GuideConstraint::SPHERICAL:
            return "spherical";
        default:
            return "unknown";
    }
}

void ChLinkMotorLinear::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorLinear>();

    // serialize parent class
    ChLinkMotor::ArchiveOut(archive_out);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinear::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorLinear>();

    // deserialize parent class
    ChLinkMotor::ArchiveIn(archive_in);

    // deserialize all member data:
}

}  // end namespace chrono
