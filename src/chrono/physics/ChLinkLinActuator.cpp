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

#include "chrono/physics/ChLinkLinActuator.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLinActuator)

ChLinkLinActuator::ChLinkLinActuator() : offset(0.1) {
    dist_funct = chrono_types::make_shared<ChFunction_Const>(0);

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    mask.SetLockMask(true, false, false, false, false, false, false);
    BuildLink();
}

ChLinkLinActuator::ChLinkLinActuator(const ChLinkLinActuator& other) : ChLinkLockLock(other) {
    offset = other.offset;
    dist_funct = std::shared_ptr<ChFunction>(other.dist_funct->Clone());
}

void ChLinkLinActuator::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLockLock::UpdateTime(mytime);

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);

    Vector mx = Vnorm(absdist);

    Vector my = ma.Get_A_Yaxis();
    if (Vequal(mx, my)) {
        if (mx.x() == 1.0)
            my = VECT_Y;
        else
            my = VECT_X;
    }
    Vector mz = Vnorm(Vcross(mx, my));
    my = Vnorm(Vcross(mz, mx));

    ma.Set_A_axis(mx, my, mz);

    Coordsys newmarkpos;
    ChVector<> oldpos = marker2->GetPos();  // backup to avoid numerical err.accumulation
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);  // rotate "main" marker2 into tangent position (may add err.accumulation)
    marker2->SetPos(oldpos);                // backup to avoid numerical err.accumulation

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x() = dist_funct->Get_y(ChTime) + offset;  // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x() = dist_funct->Get_y_dx(ChTime);  // distance speed

    deltaC_dtdt.pos = VNULL;
    deltaC_dtdt.pos.x() = dist_funct->Get_y_dxdx(ChTime);  // distance acceleration

    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    Vector tang_speed = GetRelM_dt().pos;
    tang_speed.x() = 0;                     // only z-y coords in relative tang speed vector
    double len_absdist = Vlength(absdist);  // don't divide by zero
    if (len_absdist > 1E-6)
        deltaC_dtdt.pos.x() -= pow(Vlength(tang_speed), 2) / Vlength(absdist);  // An = Adelta -(Vt^2 / r)

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkLinActuator::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkLinActuator>();

    // serialize parent class
    ChLinkLockLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(offset);
    marchive << CHNVP(dist_funct);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLinActuator::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChLinkLinActuator>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(offset);
    marchive >> CHNVP(dist_funct);
}

}  // end namespace chrono
