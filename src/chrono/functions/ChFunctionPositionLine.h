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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHFUNCTIONPOSITION_LINE_H
#define CHFUNCTIONPOSITION_LINE_H

#include "chrono/geometry/ChLine.h"
#include "chrono/functions/ChFunctionPosition.h"
#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A motion function p=f(s) where p(t) is defined with a
/// ChLine geometry object, ex. ChLineArc or ChLineBSpline etc.

class ChApi ChFunctionPositionLine : public ChFunctionPosition {
  public:
    ChFunctionPositionLine();
    ChFunctionPositionLine(const ChFunctionPositionLine& other);
    virtual ~ChFunctionPositionLine();

    /// "Virtual" copy constructor.
    virtual ChFunctionPositionLine* Clone() const override { return new ChFunctionPositionLine(*this); }

    /// Get the trajectory line
    std::shared_ptr<ChLine> GetLine() const { return m_trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void SetLine(std::shared_ptr<ChLine> line) { m_trajectory_line = line; }

    /// Gets the address of the function u=u(s) telling
    /// how the curvilinear parameter u of the spline changes in s (time).
    std::shared_ptr<ChFunction> GetSpaceFunction() const { return m_space_fun; }

    /// Sets the function u=u(s) telling how the curvilinear parameter
    /// of the spline changes in s (time).
    /// Otherwise, by default, is a linear ramp, so evaluates the spline from begin at s=0 to end at s=1
    void SetSpaceFunction(std::shared_ptr<ChFunction> funct) { m_space_fun = funct; }

    /// Return the position imposed by the function, at \a s.
    virtual ChVector3d GetPos(double s) const override;

    /// Return the linear velocity imposed by the function, at \a s.
    virtual ChVector3d GetLinVel(double s) const override;

    /// Return the linear acceleration imposed the function, at \a s.
    virtual ChVector3d GetLinAcc(double s) const override;

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChLine> m_trajectory_line;

    std::shared_ptr<ChFunction> m_space_fun;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPositionLine, 0)

}  // end namespace chrono

#endif
