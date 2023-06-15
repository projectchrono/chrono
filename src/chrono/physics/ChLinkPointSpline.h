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

#ifndef CHLINKPOINTSPLINE_H
#define CHLINKPOINTSPLINE_H

#include "chrono/geometry/ChLine.h"
#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// ChLinkPointSpline class.
/// This class implements the 'point on a spline curve' constraint.
/// It can be used also to simulate curvilinear glyphs, etc.

class ChApi ChLinkPointSpline : public ChLinkLockLock {
  public:
    ChLinkPointSpline();
    ChLinkPointSpline(const ChLinkPointSpline& other);
    ~ChLinkPointSpline() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointSpline* Clone() const override { return new ChLinkPointSpline(*this); }

    /// Get the address of the trajectory line
    std::shared_ptr<geometry::ChLine> Get_trajectory_line() const { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void Set_trajectory_line(std::shared_ptr<geometry::ChLine> mline);

    /// Set the tolerance controlling the accuracy of nearest point search (default: 1e-6)
    void SetTolerance(double tol) { tolerance = tol; }

    /// Update link at current configuration (moves the constraint main marker tangent to the line).
    virtual void UpdateTime(double mytime) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    std::shared_ptr<geometry::ChLine> trajectory_line;  ///< line for the trajectory
    double tolerance;                                   ///< tolerance for nearest point search
};

CH_CLASS_VERSION(ChLinkPointSpline, 0)

}  // end namespace chrono

#endif
