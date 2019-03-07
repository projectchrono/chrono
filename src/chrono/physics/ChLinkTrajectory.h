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

/// ChLinkTrajectory class.
/// This class implements the 'point on an imposed trajectory' constraint.
/// It can be used also to simulate the imposed motion of objects in space
/// (for motion capture, for example).

class ChApi ChLinkTrajectory : public ChLinkLockLock {

  protected:
    std::shared_ptr<ChFunction> space_fx;  ///< function providing the time history of the trajectory parameter
    std::shared_ptr<geometry::ChLine> trajectory_line;  ///< line for the trajectory.
    bool modulo_s;                                      ///< modulation

  public:
    ChLinkTrajectory();
    ChLinkTrajectory(const ChLinkTrajectory& other);
    virtual ~ChLinkTrajectory() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkTrajectory* Clone() const override { return new ChLinkTrajectory(*this); }

    /// Gets the address of the function s=s(t) telling
    /// how the curvilinear parameter of the trajectory changes in time.
    std::shared_ptr<ChFunction> Get_space_fx() const { return space_fx; }

    /// Sets the function s=s(t) telling how the curvilinear parameter
    /// of the trajectory changes in time.
    void Set_space_fx(std::shared_ptr<ChFunction> m_funct);

    /// Tells that the s in  s=s(t)  function will be wrapped in 0..1 if it is outside 0..1
    void Set_modulo_one_fx(bool mmod) { modulo_s = mmod; }

    /// Get the address of the trajectory line
    std::shared_ptr<geometry::ChLine> Get_trajectory_line() const { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void Set_trajectory_line(std::shared_ptr<geometry::ChLine> mline);

    /// Use this function after link creation, to initialize the link from
    /// two joined rigid bodies.
    /// Both rigid bodies must belong to the same ChSystem.
    /// Two markers will be created and added to the rigid bodies (later,
    /// you can use GetMarker1() and GetMarker2() to access them.
    /// Marker2 will stay in origin of body2. Trajectory is considered relative to body2.
    void Initialize(std::shared_ptr<ChBody> mbody1,  ///< first  body to join (the one that follows the trajectory)
                    std::shared_ptr<ChBody> mbody2,  ///< second body to join (the one that contains the trajectory)
                    const ChVector<>& mpos1,         ///< position of the 'following point' on body1, relative to coordinate of body1.
                    std::shared_ptr<geometry::ChLine> mline  ///< the line on mbody2 to be followed by point mpos1 of mbody1
                    );

    /// Overrides the parent class function. Here it moves the
    /// constraint mmain marker tangent to the line.
    virtual void UpdateTime(double mytime) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkTrajectory,0)

}  // end namespace chrono

#endif
