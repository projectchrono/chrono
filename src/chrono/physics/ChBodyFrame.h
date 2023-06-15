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

#ifndef CHBODYFRAME_H
#define CHBODYFRAME_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

/// Class for objects that represent moving frames in space and contain state variables.
class ChApi ChBodyFrame : public ChFrameMoving<double> {
  public:
    ChBodyFrame() {}
    ChBodyFrame(const ChBodyFrame& other) : ChFrameMoving<double>(other) {}

    virtual ~ChBodyFrame() {}

    /// Return a reference to the encapsulated variables, representing states (pos, speed or accel.) and forces.
    virtual ChVariables& Variables() = 0;

    /// Transform a force applied at a point on the body  into a force and moment applied to the COM and expressed in
    /// the absolute frame.
    /// If local = true, the provided applied force and point are assumed to be expressed in body coordinates.
    /// If local = false, the provided applied force and point is assumed to be expressed in absolute coordinates.
    void To_abs_forcetorque(const ChVector<>& force,
                            const ChVector<>& appl_point,
                            bool local,
                            ChVector<>& resultforce,
                            ChVector<>& resulttorque);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChBodyFrame,0)


}  // end namespace chrono

#endif
