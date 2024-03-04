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

#ifndef CHFUNCT_POLY23_H
#define CHFUNCT_POLY23_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Cubic smooth step.
///     `y(x) = ampl * (x - x_start)^2 * (x - x_end)^2 * (3 - 2*(x - x_start))`
/// with `x` in `[x_start, x_end]`.
/// First derivatives at the start and end points are zero.
class ChApi ChFunctionPoly23 : public ChFunction {
  private:
    double m_ampl;
    double m_x_start;
    double m_x_end;

  public:
    ChFunctionPoly23() : m_ampl(1.0), m_x_start(0.0), m_x_end(1.0) {}
    ChFunctionPoly23(double ampl, double x_start, double x_end) : m_ampl(ampl), m_x_start(x_start), m_x_end(x_end) {}
    ChFunctionPoly23(const ChFunctionPoly23& other);
    ~ChFunctionPoly23() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionPoly23* Clone() const override { return new ChFunctionPoly23(*this); }

    virtual Type GetType() const override { return ChFunction::Type::POLY23; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    /// Set the start of the step: GetVal(x_start) = 0.
    void SetStartArg(double x_start) { m_x_start = x_start; }

    /// Set the end of the step: GetVal(x_end) = amplitude.
    void SetEndArg(double x_end) { m_x_end = x_end; }

    /// Set the value of the function at the end of the step.
    void SetAmplitude(double ampl) { m_ampl = ampl; }

    /// Get the start of the step: GetVal(x_start) = 0.
    double GetStart() const { return m_x_start; }

    /// Get the end of the step: GetVal(x_end) = amplitude.
    double GetEnd() const { return m_x_end; }

    /// Get the value of the function at the end of the step.
    double GetAmplitude() const { return m_ampl; }

    virtual double GetPositiveAccelerationCoeff() const override { return 6.0; }
    virtual double GetNegativeAccelerationCoeff() const override { return 6.0; }
    virtual double GetVelocityCoefficient() const override { return 1.5; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPoly23, 0)

}  // end namespace chrono

#endif
