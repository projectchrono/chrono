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

#include <vector>

#include "chrono/core/ChFrameMoving.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

/// Class for objects that represent moving frames in space and contain state variables.
/// Such objects (rigid bodies and FEA nodes with rotational DOFs) also provide force and torque accumulators.
class ChApi ChBodyFrame : public ChFrameMoving<double> {
  public:
    ChBodyFrame() {}

    /// Copy constructor.
    /// Force and torque accumulators are not copied (they are owned by the code that created them).
    ChBodyFrame(const ChBodyFrame& other) : ChFrameMoving<double>(other) {}

    virtual ~ChBodyFrame() {}

    /// Assignment operator.
    /// Force and torque accumulators are not copied (they are owned by the code that created them).
    ChBodyFrame& operator=(const ChBodyFrame& other) {
        ChFrameMoving<double>::operator=(other);
        return *this;
    }

    using ChFrameMoving<double>::operator=;

    /// Return a reference to the encapsulated variables, representing states (pos, speed or accel.) and forces.
    virtual ChVariables& Variables() = 0;

    /// Add a new force and torque accumulator.
    /// An arbitrary number of accumulators can be specified for an object, differentiated by their index `idx` (as
    /// returned by this function). At each step in a simulation loop, a typical use of these accumulators is:
    /// <pre>
    ///    EmptyAccumulator(idx);
    ///    AccumulateForce(idx, ...);
    ///    AccumulateTorque(idx, ...);
    ///    ...
    ///    AccumulateForce(idx, ...);
    ///    ...
    /// </pre>
    unsigned int AddAccumulator();

    /// Clear the accumulator with specified index.
    void EmptyAccumulator(unsigned int idx);

    /// Include a concentrated force in the specified accumulator.
    /// The accumulator force and torque are incremented with the specified applied force and induced moment at the
    /// frame origin. It is the caller's responsibility to clear the force and torque accumulator before reuse (e.g., at
    /// each integration step). If local = true, the provided applied force and application point are assumed to be
    /// expressed in local coordinates; otherwise (local = false), these quantities are assumed to be expressed in
    /// absolute coordinates.
    void AccumulateForce(unsigned int idx,              ///< index of the accumulator
                         const ChVector3d& force,       ///< applied force
                         const ChVector3d& appl_point,  ///< application point
                         bool local                     ///< force and point expressed in local frame?
    );

    /// Include a torque in the specified accumulator.
    /// The accumulator torque is incremented with the specified torque. It is the caller's responsibility to clear the
    /// force and torque accumulator before reuse (e.g., at each integration step). If local = true, the provided
    /// applied torque is assumed to be expressed in local coordinates; otherwise (local = false), it is assumed to be
    /// expressed in absolute coordinates.
    void AccumulateTorque(unsigned int idx,          ///< index of the accumulator
                          const ChVector3d& torque,  ///< applied torque
                          bool local                 ///< torque expressed in local frame?
    );

    /// Return the current value of the accumulated force from the specified accumulator.
    /// Note that this is a resultant force as applied to the frame origin and expressed in the absolute frame.
    const ChVector3d& GetAccumulatedForce(unsigned int idx) const;

    /// Return the current value of the accumulated torque from the specified accumulator.
    /// Note that this is a resultant torque expressed in the local frame.
    const ChVector3d& GetAccumulatedTorque(unsigned int idx) const;

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

  protected:
    struct WrenchAccumulator {
        ChVector3d force;   ///< force accumulator, applied at frame origin (in absolute coords)
        ChVector3d torque;  ///< torque accumulator (in local coords)
    };

    std::vector<WrenchAccumulator> accumulators;  ///< force and torque accumulators
};

CH_CLASS_VERSION(ChBodyFrame, 0)

}  // end namespace chrono

#endif
