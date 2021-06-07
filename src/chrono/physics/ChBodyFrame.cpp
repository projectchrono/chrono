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

void ChBodyFrame::To_abs_forcetorque(const ChVector<>& force,
                                     const ChVector<>& appl_point,
                                     bool local,
                                     ChVector<>& resultforce,
                                     ChVector<>& resulttorque) {
    if (local) {
        resultforce = TransformDirectionLocalToParent(force);
        resulttorque = Vcross(TransformDirectionLocalToParent(appl_point), resultforce);
    } else {
        resultforce = force;
        resulttorque = Vcross(Vsub(appl_point, coord.pos), force);
    }
}

void ChBodyFrame::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChBodyFrame>();

    // serialize parent class
    ChFrameMoving<double>::ArchiveOUT(marchive);
}

void ChBodyFrame::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChBodyFrame>();

    // deserialize parent class
    ChFrameMoving<double>::ArchiveIN(marchive);
}

}  // end namespace chrono