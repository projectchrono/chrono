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
// Authors: Dario Fusai
// =============================================================================

#ifndef CHFUNCT_CYCLOIDAL_H
#define CHFUNCT_CYCLOIDAL_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Ramp function, as cycloidal:
///   - h   = height, amount of displacement
///   - end = duration of motion
class ChApi ChFunction_Cycloidal : public ChFunction {
public:
    ChFunction_Cycloidal() : h(1), end(1) {}

    ChFunction_Cycloidal(double m_h, double m_end);

    ChFunction_Cycloidal(const ChFunction_Cycloidal& other);

    ~ChFunction_Cycloidal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Cycloidal* Clone() const override { return new ChFunction_Cycloidal(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_CYCLOIDAL; }

    virtual double Get_y(double x) const override;

    virtual double Get_y_dx(double x) const override;

    virtual double Get_y_dxdx(double x) const override;

    virtual double Get_y_dxdxdx(double x) const override;

    void Set_end(double m_end);

    void Set_h(double m_h) { h = m_h; }

    double Get_end() const { return end; }

    double Get_h() const { return h; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

private:
    double h;   ///< total amount of displacement
    double end; ///< duration of motion
};

/// @} chrono_functions

}  // end namespace chrono

#endif
