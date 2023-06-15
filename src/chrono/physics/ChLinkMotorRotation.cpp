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

#include "chrono/physics/ChLinkMotorRotation.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLinkMotorRotation)  NO! ABSTRACT!

ChLinkMotorRotation::ChLinkMotorRotation() {
    this->SetSpindleConstraint(SpindleConstraint::REVOLUTE);

    mrot = 0;
    mrot_dt = 0;
    mrot_dtdt = 0;
}

ChLinkMotorRotation::ChLinkMotorRotation(const ChLinkMotorRotation& other) : ChLinkMotor(other) {
    mrot = other.mrot;
    mrot_dt = other.mrot_dt;
    mrot_dtdt = other.mrot_dtdt;
}

ChLinkMotorRotation::~ChLinkMotorRotation() {}

void ChLinkMotorRotation::SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry) {
    this->c_x = mc_x;
    this->c_y = mc_y;
    this->c_z = mc_z;
    this->c_rx = mc_rx;
    this->c_ry = mc_ry;
    SetupLinkMask();
}

void ChLinkMotorRotation::SetSpindleConstraint(const SpindleConstraint mconstraint) {
    if (mconstraint == SpindleConstraint::FREE) {
        this->c_x = false;
        this->c_y = false;
        this->c_z = false;
        this->c_rx = false;
        this->c_ry = false;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::REVOLUTE) {
        this->c_x = true;
        this->c_y = true;
        this->c_z = true;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::CYLINDRICAL) {
        this->c_x = true;
        this->c_y = true;
        this->c_z = false;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::OLDHAM) {
        this->c_x = false;
        this->c_y = false;
        this->c_z = false;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
}

void ChLinkMotorRotation::Update(double mytime, bool update_assets) {
    // Inherit parent class:
    ChLinkMotor::Update(mytime, update_assets);

    // compute aux data for future reference (istantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(this->frame1) >> (ChFrameMoving<>)(*this->Body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(this->frame2) >> (ChFrameMoving<>)(*this->Body2);
    ChFrameMoving<> aframe12;
    aframe2.TransformParentToLocal(aframe1, aframe12);

    // multi-turn rotation code
    double last_totrot = this->mrot;
    double last_rot = remainder(last_totrot, CH_C_2PI);
    double last_turns = last_totrot - last_rot;
    double new_rot = remainder(aframe12.GetRot().Q_to_Rotv().z(), CH_C_2PI);
    this->mrot = last_turns + new_rot;
    if (fabs(new_rot + CH_C_2PI - last_rot) < fabs(new_rot - last_rot))
        this->mrot = last_turns + new_rot + CH_C_2PI;
    if (fabs(new_rot - CH_C_2PI - last_rot) < fabs(new_rot - last_rot))
        this->mrot = last_turns + new_rot - CH_C_2PI;

    this->mrot_dt = aframe12.GetWvel_loc().z();
    this->mrot_dtdt = aframe12.GetWacc_loc().z();
}

void ChLinkMotorRotation::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotation>();

    // serialize parent class
    ChLinkMotor::ArchiveOut(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotation::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorRotation>();

    // deserialize parent class
    ChLinkMotor::ArchiveIn(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
