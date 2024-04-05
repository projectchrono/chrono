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

#include "chrono/physics/ChBodyFrame.h"

namespace chrono {

ChWrenchd ChBodyFrame::AppliedForceLocalToWrenchParent(const ChVector3d& force, const ChVector3d& appl_point) {
    auto force_abs = TransformDirectionLocalToParent(force);
    return {force_abs, Vcross(TransformDirectionLocalToParent(appl_point), force_abs)};
}

ChWrenchd ChBodyFrame::AppliedForceParentToWrenchParent(const ChVector3d& force, const ChVector3d& appl_point) {
    return {force, Vcross(appl_point - GetPos(), force)};
}

void ChBodyFrame::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBodyFrame>();

    // serialize parent class
    ChFrameMoving<double>::ArchiveOut(archive_out);
}

void ChBodyFrame::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChBodyFrame>();

    // deserialize parent class
    ChFrameMoving<double>::ArchiveIn(archive_in);
}

}  // end namespace chrono