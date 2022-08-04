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

ChLinkMotorLinear::ChLinkMotorLinear() {
    this->SetGuideConstraint(GuideConstraint::PRISMATIC);

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

void ChLinkMotorLinear::SetGuideConstraint(bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
    this->c_y = mc_y;
    this->c_z = mc_z;
    this->c_rx = mc_rx;
    this->c_ry = mc_ry;
    this->c_rz = mc_rz;
    SetupLinkMask();
}

void ChLinkMotorLinear::SetGuideConstraint(const GuideConstraint mconstraint) {
    if (mconstraint == GuideConstraint::FREE) {
        this->c_y = false;
        this->c_z = false;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::PRISMATIC) {
        this->c_y = true;
        this->c_z = true;
        this->c_rx = true;
        this->c_ry = true;
        this->c_rz = true;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::SPHERICAL) {
        this->c_y = true;
        this->c_z = true;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }
}

void ChLinkMotorLinear::Update(double mytime, bool update_assets) {
    // Inherit parent class:
    ChLinkMotor::Update(mytime, update_assets);

    // compute aux data for future reference (istantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(this->frame1) >> (ChFrameMoving<>)(*this->Body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(this->frame2) >> (ChFrameMoving<>)(*this->Body2);
    ChFrameMoving<> aframe12;
    aframe2.TransformParentToLocal(aframe1, aframe12);

    //// RADU TODO: revisit this.
    //// This is incorrect for GuideConstraint::FREE
    //// Should use something like sqrt(Vdot(relpos,relpos)), but taking into account sign?

    this->mpos = aframe12.GetPos().x();
    this->mpos_dt = aframe12.GetPos_dt().x();
    this->mpos_dtdt = aframe12.GetPos_dtdt().x();
}

void ChLinkMotorLinear::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinear>();

    // serialize parent class
    ChLinkMotor::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinear::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorLinear>();

    // deserialize parent class
    ChLinkMotor::ArchiveIN(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
