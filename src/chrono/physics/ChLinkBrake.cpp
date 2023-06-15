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

#include "chrono/physics/ChLinkBrake.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkBrake)

ChLinkBrake::ChLinkBrake()
    : brake_torque(0), stick_ratio(1.1), brake_mode(BRAKE_ROTATION), last_dir(0), must_stick(false) {
    // Mask: initialize our LinkMaskLF (lock formulation mask)
    mask.SetLockMask(false, false, false, false, false, false, false);
    BuildLink();
}

ChLinkBrake::ChLinkBrake(const ChLinkBrake& other) : ChLinkLock(other) {
    brake_torque = other.brake_torque;
    stick_ratio = other.stick_ratio;
    brake_mode = other.brake_mode;

    last_dir = other.last_dir;
    must_stick = other.must_stick;
}

void ChLinkBrake::Set_brake_mode(int mmode) {
    if (mmode != brake_mode) {
        brake_mode = mmode;

        // reset mask for default free brake
        mask.Constr_E3().SetMode(CONSTRAINT_FREE);
        mask.Constr_X().SetMode(CONSTRAINT_FREE);
        BuildLink();
    }
}

void ChLinkBrake::SetDisabled(bool mdis) {
    ChLinkLock::SetDisabled(mdis);

    mask.Constr_E3().SetMode(CONSTRAINT_FREE);
    mask.Constr_X().SetMode(CONSTRAINT_FREE);
    BuildLink();
}

// Update time: just change internal time!
void ChLinkBrake::UpdateTime(double time) {
    ChTime = time;
}

// Update forces: if not sticked, apply torque
void ChLinkBrake::UpdateForces(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

    if (this->IsDisabled())
        return;

    // then, if not sticking,
    if (this->brake_torque) {
        if (brake_mode == BRAKE_ROTATION) {
            if (mask.Constr_E3().IsActive() == false) {
                int mdir;

                Vector mv_torque = Vmul(VECT_Z, this->brake_torque);
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
        if (brake_mode == BRAKE_TRANSLATEX) {
            if (mask.Constr_X().IsActive() == false) {
                int mdir;

                Vector mv_force = Vmul(VECT_X, this->brake_torque);
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

void ChLinkBrake::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkBrake>();

    // serialize parent class
    ChLinkLock::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(brake_torque);
    marchive << CHNVP(stick_ratio);
    marchive << CHNVP(brake_mode);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkBrake::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkBrake>();

    // deserialize parent class
    ChLinkLock::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(brake_torque);
    marchive >> CHNVP(stick_ratio);
    marchive >> CHNVP(brake_mode);
}

}  // end namespace chrono
