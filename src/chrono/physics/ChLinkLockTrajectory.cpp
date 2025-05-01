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

#include "chrono/geometry/ChLineSegment.h"
#include "chrono/physics/ChLinkLockTrajectory.h"

namespace chrono {

static const double FD_STEP_HIGH = 1e-4;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockTrajectory)

ChLinkLockTrajectory::ChLinkLockTrajectory() : modulo_s(false) {
    // initializes type
    type = Type::TRAJECTORY;

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);

    // default trajectory is a segment
    trajectory_line = chrono_types::make_shared<ChLineSegment>();

    // Mask: initialize our LinkMaskLF (lock formulation mask) to X  only
    mask.SetLockMask(true, true, true, false, false, false, false);

    BuildLink();
}

ChLinkLockTrajectory::ChLinkLockTrajectory(const ChLinkLockTrajectory& other) : ChLinkLockLock(other) {
    space_fx = std::shared_ptr<ChFunction>(other.space_fx->Clone());                     // deep copy
    trajectory_line = std::shared_ptr<ChLine>((ChLine*)other.trajectory_line->Clone());  // deep copy
}

void ChLinkLockTrajectory::SetTimeLaw(std::shared_ptr<ChFunction> m_funct) {
    space_fx = m_funct;
}

void ChLinkLockTrajectory::SetTrajectory(std::shared_ptr<ChLine> mline) {
    trajectory_line = mline;
}

void ChLinkLockTrajectory::UpdateTime(double time) {
    ChLinkLock::UpdateTime(time);

    double tstep = FD_STEP_HIGH;
    double tr_time = space_fx->GetVal(time);
    double tr_timeB = space_fx->GetVal(time + tstep);
    double tr_timeA = space_fx->GetVal(time - tstep);

    if (trajectory_line) {
        if (modulo_s) {
            tr_time = fmod(tr_time, 1);
            tr_timeA = fmod(tr_timeA, 1);
            tr_timeB = fmod(tr_timeB, 1);
        }
        auto result = trajectory_line->Evaluate(tr_time);
        auto resultA = trajectory_line->Evaluate(tr_timeA);
        auto resultB = trajectory_line->Evaluate(tr_timeB);

        // if line coordinate is relative to body2:
        marker2->ImposeRelativeTransform(ChFrame<>());
        deltaC.pos = result;
        deltaC_dt.pos = (resultB - resultA) * (1 / (2 * tstep));
        deltaC_dtdt.pos = (resultA + resultB - result * 2) * (4 / std::pow(2 * tstep, 2));
        /*
        // if line coordinate is relative to absolute space:
        ChMatrix33<> mw(marker2->GetAbsCoordsys().rot);
        deltaC.pos = mw.transpose() * (result - marker2->GetAbsCoordsys().pos);  //// CORRECT?
        deltaC_dt.pos = mw.transpose() * ((resultB - resultA) * (1 / (2 * tstep)));
        deltaC_dtdt.pos = mw.transpose() * ((resultA + resultB - result * 2) * (4 / std::pow(2 * tstep, 2)));
        */

        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
    } else {
        std::cerr << "NO TRAJECTORY" << std::endl;
    }
}

void ChLinkLockTrajectory::Initialize(std::shared_ptr<ChBody> body1,
                                      std::shared_ptr<ChBody> body2,
                                      const ChVector3d& pos1,
                                      std::shared_ptr<ChLine> line) {
    ChLinkMarkers::Initialize(body1, body2, true, ChFrame<>(pos1), ChFrame<>());
    this->SetTrajectory(line);
}

void ChLinkLockTrajectory::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockTrajectory>();

    // serialize parent class
    ChLinkLockLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(space_fx);
    archive_out << CHNVP(trajectory_line);
    archive_out << CHNVP(modulo_s);
}

void ChLinkLockTrajectory::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockTrajectory>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(space_fx);
    archive_in >> CHNVP(trajectory_line);
    archive_in >> CHNVP(modulo_s);
}

}  // end namespace chrono
