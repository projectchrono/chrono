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

#ifndef CHFUNCTIONPOSITION_XYZFUNCTIONS_H
#define CHFUNCTIONPOSITION_XYZFUNCTIONS_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionPosition.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A motion function p=f(s) where p(t) is defined with three
/// independent ChFunction objects, each for px, py, pz component.

class ChApi ChFunctionPositionXYZFunctions : public ChFunctionPosition {
  public:
    ChFunctionPositionXYZFunctions();
    ChFunctionPositionXYZFunctions(const ChFunctionPositionXYZFunctions& other);
    virtual ~ChFunctionPositionXYZFunctions();

    /// "Virtual" copy constructor.
    virtual ChFunctionPositionXYZFunctions* Clone() const override { return new ChFunctionPositionXYZFunctions(*this); }

    /// Set the fx(s) function for the X component of the motion, ie. p.x = fx(s)
    void SetFunctionX(std::shared_ptr<ChFunction> mx) { this->px = mx; }
    /// Get the fx(s) function for the X component of the motion, ie. p.x = fx(s)
    std::shared_ptr<ChFunction> GetFunctionX() { return this->px; }
    /// Set the fy(s) function for the Y component of the motion, ie. p.y = fy(s)
    void SetFunctionY(std::shared_ptr<ChFunction> my) { this->py = my; }
    /// Get the fy(s) function for the Y component of the motion, ie. p.y = fy(s)
    std::shared_ptr<ChFunction> GetFunctionY() { return this->py; }
    /// Set the fz(s) function for the Z component of the motion, ie. p.z = fz(s)
    void SetFunctionZ(std::shared_ptr<ChFunction> mz) { this->pz = mz; }
    /// Get the fz(s) function for the Z component of the motion, ie. p.z = fz(s)
    std::shared_ptr<ChFunction> GetFunctionZ() { return this->pz; }

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
    std::shared_ptr<ChFunction> px;
    std::shared_ptr<ChFunction> py;
    std::shared_ptr<ChFunction> pz;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPositionXYZFunctions, 0)

}  // end namespace chrono

#endif
