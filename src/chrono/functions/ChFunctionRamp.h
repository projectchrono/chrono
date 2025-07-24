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

#ifndef CHFUNCT_RAMP_H
#define CHFUNCT_RAMP_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Ramp function.
/// `y = y0 + m * x`
class ChApi ChFunctionRamp : public ChFunction {
  public:
    ChFunctionRamp() : m_y0(0), m_ang_coeff(1) {}
    ChFunctionRamp(double y0, double ang_coeff) : m_y0(y0), m_ang_coeff(ang_coeff) {}
    ChFunctionRamp(const ChFunctionRamp& other);
    ~ChFunctionRamp() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionRamp* Clone() const override { return new ChFunctionRamp(*this); }

    virtual Type GetType() const override { return ChFunction::Type::RAMP; }

    virtual double GetVal(double x) const override { return (m_y0 + (x * m_ang_coeff)); }
    virtual double GetDer(double x) const override { return (m_ang_coeff); }
    virtual double GetDer2(double x) const override { return 0; }
    virtual double GetDer3(double x) const override { return 0; }

    /// Set the initial value.
    void SetStartVal(double y0) { m_y0 = y0; }

    /// Get the initial value.
    double GetStartVal() { return m_y0; }

    /// Set the angular coefficient.
    void SetAngularCoeff(double ang_coeff) { m_ang_coeff = ang_coeff; }

    /// Get the angular coefficient.
    double GetAngularCoeff() { return m_ang_coeff; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double m_y0;
    double m_ang_coeff;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRamp, 0)

}  // end namespace chrono

#endif
