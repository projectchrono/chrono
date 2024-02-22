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

void ChBodyFrame::To_abs_forcetorque(const ChVector3d& force,
                                     const ChVector3d& appl_point,
                                     bool local,
                                     ChVector3d& resultforce,
                                     ChVector3d& resulttorque) {
    if (local) {
        resultforce = TransformDirectionLocalToParent(force);
        resulttorque = Vcross(TransformDirectionLocalToParent(appl_point), resultforce);
    } else {
        resultforce = force;
        resulttorque = Vcross(Vsub(appl_point, GetPos()), force);
    }
}

void ChBodyFrame::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBodyFrame>();

    // serialize parent class
    ChFrameMoving<double>::ArchiveOut(archive_out);
}

void ChBodyFrame::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChBodyFrame>();

    // deserialize parent class
    ChFrameMoving<double>::ArchiveIn(archive_in);
}

}  // end namespace chrono