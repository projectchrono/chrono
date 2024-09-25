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

#include <cmath>

#include "chrono/physics/ChLinkLockLinActuator.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockLinActuator)

ChLinkLockLinActuator::ChLinkLockLinActuator() : offset(0.1) {
    dist_funct = chrono_types::make_shared<ChFunctionConst>(0);

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    mask.SetLockMask(true, false, false, false, false, false, false);
    BuildLink();
}

ChLinkLockLinActuator::ChLinkLockLinActuator(const ChLinkLockLinActuator& other) : ChLinkLockLock(other) {
    offset = other.offset;
    dist_funct = std::shared_ptr<ChFunction>(other.dist_funct->Clone());
}

void ChLinkLockLinActuator::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLockLock::UpdateTime(mytime);

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);

    ChMatrix33<> ma(marker2->GetAbsCoordsys().rot);

    ChVector3d absdist = marker1->GetAbsCoordsys().pos - marker2->GetAbsCoordsys().pos;

    ChVector3d mx = Vnorm(absdist);

    ChVector3d my = ma.GetAxisY();
    if (Vequal(mx, my)) {
        if (mx.x() == 1.0)
            my = VECT_Y;
        else
            my = VECT_X;
    }
    ChVector3d mz = Vnorm(Vcross(mx, my));
    my = Vnorm(Vcross(mz, mx));

    ma.SetFromDirectionAxes(mx, my, mz);

    // backup to avoid numerical err.accumulation
    ChVector3d oldpos = marker2->GetPos();
    // rotate "main" marker2 into tangent position
    marker2->ImposeAbsoluteTransform(ChFrame<>(marker2->GetAbsCoordsys().pos, ma.GetQuaternion()));
    // backup to avoid numerical err.accumulation
    marker2->SetPos(oldpos);

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x() = dist_funct->GetVal(ChTime) + offset;  // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x() = dist_funct->GetDer(ChTime);  // distance speed

    deltaC_dtdt.pos = VNULL;
    deltaC_dtdt.pos.x() = dist_funct->GetDer2(ChTime);  // distance acceleration

    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    ChVector3d tang_speed = GetRelCoordsysDt().pos;
    tang_speed.x() = 0;                     // only z-y coords in relative tang speed vector
    double len_absdist = Vlength(absdist);  // don't divide by zero
    if (len_absdist > 1E-6)
        deltaC_dtdt.pos.x() -= std::pow(Vlength(tang_speed), 2) / Vlength(absdist);  // An = Adelta -(Vt^2 / r)

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkLockLinActuator::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockLinActuator>();

    // serialize parent class
    ChLinkLockLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(offset);
    archive_out << CHNVP(dist_funct);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockLinActuator::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockLinActuator>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(offset);
    archive_in >> CHNVP(dist_funct);
}

}  // end namespace chrono
