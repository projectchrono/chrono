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

    /// Transform a force applied to a point on the body to a force and moment at the frame origin.
    /// The applied force and its application point are assumed to be expressed in the body frame.
    /// The resulting force and torque are expressed in the parent frame.
    ChWrenchd AppliedForceLocalToWrenchParent(const ChVector3d& force,      ///< applied force, in local coords.
                                              const ChVector3d& appl_point  ///< application point, in local coords
    );

    /// Transform a force applied to a point on the body to a force and moment at the frame origin.
    /// The applied force and its application point are assumed to be expressed in the parent frame.
    /// The resulting force and torque are expressed in the parent frame.
    ChWrenchd AppliedForceParentToWrenchParent(const ChVector3d& force,      ///< applied force, in abs. coords.
                                               const ChVector3d& appl_point  ///< application point, in abs. coords
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChBodyFrame, 0)

}  // end namespace chrono

#endif
