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

ChLinkMotorLinear::ChLinkMotorLinear() : m_actuated_idx(0), mpos(0), mpos_dt(0), mpos_dtdt(0) {
    SetGuideConstraint(GuideConstraint::PRISMATIC);

    // DEVELOPER NOTES: c_z flag should be set by derived classes according to the type of constraint
    //                  e.g. force constraints have c_z=false since no proper constraint should be added
}

ChLinkMotorLinear::ChLinkMotorLinear(const ChLinkMotorLinear& other) : ChLinkMotor(other) {
    mpos = other.mpos;
    mpos_dt = other.mpos_dt;
    mpos_dtdt = other.mpos_dtdt;
}

ChLinkMotorLinear::~ChLinkMotorLinear() {}

void ChLinkMotorLinear::SetGuideConstraint(bool cx, bool cy, bool crx, bool cry, bool crz) {
    c_x = cx;
    c_y = cy;
    c_rx = crx;
    c_ry = cry;
    c_rz = crz;
    SetupLinkMask();

    m_actuated_idx = (int)c_x + (int)c_y;
}

void ChLinkMotorLinear::SetGuideConstraint(const GuideConstraint constraint) {
    if (constraint == GuideConstraint::FREE) {
        c_x = false;
        c_y = false;
        c_rx = false;
        c_ry = false;
        c_rz = false;
        SetupLinkMask();
    }
    if (constraint == GuideConstraint::PRISMATIC) {
        c_x = true;
        c_y = true;
        c_rx = true;
        c_ry = true;
        c_rz = true;
        SetupLinkMask();
    }
    if (constraint == GuideConstraint::SPHERICAL) {
        c_x = true;
        c_y = true;
        c_rx = false;
        c_ry = false;
        c_rz = false;
        SetupLinkMask();
    }

    m_actuated_idx = (int)c_x + (int)c_y;
}

void ChLinkMotorLinear::Update(double time, UpdateFlags update_flags) {
    // Inherit parent class:
    ChLinkMotor::Update(time, update_flags);

    // compute aux data for future reference (instantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(m_frame1) >> (ChFrameMoving<>)(*m_body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(m_frame2) >> (ChFrameMoving<>)(*m_body2);
    ChFrameMoving<> aframe12 = aframe2.TransformParentToLocal(aframe1);

    //// RADU TODO: revisit this.
    //// This is incorrect for GuideConstraint::FREE
    //// Should use something like sqrt(Vdot(relpos,relpos)), but taking into account sign?

    mpos = aframe12.GetPos().z();
    mpos_dt = aframe12.GetPosDt().z();
    mpos_dtdt = aframe12.GetPosDt2().z();
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
