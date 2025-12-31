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

#include "chrono/physics/ChLinkLockBrake.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockBrake)

class ChLinkLockBrake_Mode_enum_mapper : public ChLinkLockBrake {
  public:
    CH_ENUM_MAPPER_BEGIN(Mode);
    CH_ENUM_VAL(Mode::ROTATION);
    CH_ENUM_VAL(Mode::TRANSLATEX);
    CH_ENUM_MAPPER_END(Mode);
};

ChLinkLockBrake::ChLinkLockBrake()
    : brake_torque(0), stick_ratio(1.1), brake_mode(Mode::ROTATION), last_dir(0), must_stick(false) {
    // Mask: initialize our LinkMaskLF (lock formulation mask)
    mask.SetLockMask(false, false, false, false, false, false, false);
    BuildLink();
}

ChLinkLockBrake::ChLinkLockBrake(const ChLinkLockBrake& other) : ChLinkLock(other) {
    brake_torque = other.brake_torque;
    stick_ratio = other.stick_ratio;
    brake_mode = other.brake_mode;

    last_dir = other.last_dir;
    must_stick = other.must_stick;
}

void ChLinkLockBrake::SetBrakeMode(Mode mmode) {
    if (mmode != brake_mode) {
        brake_mode = mmode;

        // reset mask for default free brake
        mask.Constr_E3().SetMode(ChConstraint::Mode::FREE);
        mask.Constr_X().SetMode(ChConstraint::Mode::FREE);
        BuildLink();
    }
}

void ChLinkLockBrake::SetDisabled(bool mdis) {
    ChLinkLock::SetDisabled(mdis);

    mask.Constr_E3().SetMode(ChConstraint::Mode::FREE);
    mask.Constr_X().SetMode(ChConstraint::Mode::FREE);
    BuildLink();
}

// Update forces: if not sticked, apply torque
void ChLinkLockBrake::UpdateForces(double time) {
    // First, inherit to parent class
    ChLinkLock::UpdateForces(time);

    if (this->IsDisabled())
        return;

    // then, if not sticking,
    if (this->brake_torque) {
        if (brake_mode == Mode::ROTATION) {
            if (mask.Constr_E3().IsActive() == false) {
                int mdir;

                ChVector3d mv_torque = Vmul(VECT_Z, this->brake_torque);
                mdir = 0;  // clockwise torque

                if (Vdot(this->relWvel, mv_torque) > 0.0) {
                    mv_torque = Vmul(mv_torque, -1.0);  // keep torque always opposed to ang speed.
                    mdir = 1;                           // counterclockwise torque
                }

                if (mdir != this->last_dir)
                    this->must_stick = true;
                this->last_dir = mdir;

                // +++ADD TO LINK TORQUE VECTOR
                C_torque = Vadd(C_torque, mv_torque);
            }
        }
        if (brake_mode == Mode::TRANSLATEX) {
            if (mask.Constr_X().IsActive() == false) {
                int mdir;

                ChVector3d mv_force = Vmul(VECT_X, this->brake_torque);
                mdir = 0;  // F-->  rear motion: frontfacing break force

                if (this->relM_dt.pos.x() > 0.0) {
                    mv_force = Vmul(mv_force, -1.0);  // break force always opposed to speed
                    mdir = 1;                         // F<-- backfacing breakforce for front motion
                }

                if (mdir != this->last_dir)
                    this->must_stick = true;
                this->last_dir = mdir;

                // +++ADD TO LINK TORQUE VECTOR
                C_force = Vadd(C_force, mv_force);
            }
        }
    }

    // turn off sticking feature if stick ration not > 1.0
    if (this->stick_ratio <= 1.0)
        must_stick = false;
}

void ChLinkLockBrake::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockBrake>();

    // serialize parent class
    ChLinkLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(brake_torque);
    archive_out << CHNVP(stick_ratio);

    ChLinkLockBrake_Mode_enum_mapper::Mode_mapper typemapper;
    archive_out << CHNVP(typemapper(brake_mode), "ChLinkLockBrake__Mode");
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockBrake::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockBrake>();

    // deserialize parent class
    ChLinkLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(brake_torque);
    archive_in >> CHNVP(stick_ratio);

    ChLinkLockBrake_Mode_enum_mapper::Mode_mapper typemapper;
    archive_in >> CHNVP(typemapper(brake_mode), "ChLinkLockBrake__Mode");
}

}  // end namespace chrono
