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

#ifndef CHLINKTRAJECTORY_H
#define CHLINKTRAJECTORY_H

#include "chrono/geometry/ChLine.h"
#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// ChLinkLockTrajectory class.
/// This class implements the 'point on an imposed trajectory' constraint.
/// It can be used also to simulate the imposed motion of objects in space
/// (for motion capture, for example).

class ChApi ChLinkLockTrajectory : public ChLinkLockLock {
  protected:
    std::shared_ptr<ChFunction> space_fx;     ///< function providing the time history of the trajectory parameter
    std::shared_ptr<ChLine> trajectory_line;  ///< line for the trajectory.
    bool modulo_s;                            ///< modulation

  public:
    ChLinkLockTrajectory();
    ChLinkLockTrajectory(const ChLinkLockTrajectory& other);
    virtual ~ChLinkLockTrajectory() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockTrajectory* Clone() const override { return new ChLinkLockTrajectory(*this); }

    /// Gets the address of the function s=s(t) telling
    /// how the curvilinear parameter of the trajectory changes in time.
    std::shared_ptr<ChFunction> GetTimeLaw() const { return space_fx; }

    /// Sets the function s=s(t) telling how the curvilinear parameter
    /// of the trajectory changes in time.
    void SetTimeLaw(std::shared_ptr<ChFunction> m_funct);

    /// Wrap the value of s in s=s(t) within the range 0..1.
    void WrapTimeLaw(bool mmod) { modulo_s = mmod; }

    /// Get the address of the trajectory line
    std::shared_ptr<ChLine> GetTrajectory() const { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void SetTrajectory(std::shared_ptr<ChLine> mline);

    /// Initialize the link to join two rigid bodies.
    /// Both rigid bodies must belong to the same system. Two markers will be created and added to the rigid bodies.
    /// Marker2 will stay in origin of body2. Trajectory is considered relative to body2.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first  body to join (the one that follows the trajectory)
                    std::shared_ptr<ChBody> body2,  ///< second body to join (the one that contains the trajectory)
                    const ChVector3d& pos1,         ///< position of the 'following point' on body1, relative to body1
                    std::shared_ptr<ChLine> line    ///< the line on body2 to be followed by the point on body1
    );

    /// Update time-dependent quantities: move the constraint main marker tangent to the line.
    virtual void UpdateTime(double time) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    using ChLinkMarkers::Initialize;
};

CH_CLASS_VERSION(ChLinkLockTrajectory, 0)

}  // end namespace chrono

#endif
