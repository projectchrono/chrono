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

#ifndef CHFUNCT_POLY345_H
#define CHFUNCT_POLY345_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Polynomial step function
///     `y = h * (10*(x/w)^3 - 15*(x/w)^4 + 6*(x/w)^5)`
/// where:
/// - `w` is the ramp width
/// - `h` is the ramp height
///
/// First and second derivatives at the start and end points are zero.
class ChApi ChFunctionPoly345 : public ChFunction {
  private:
    double m_height;
    double m_width;

  public:
    ChFunctionPoly345() : m_height(1), m_width(1) {}
    ChFunctionPoly345(double height, double width);
    ChFunctionPoly345(const ChFunctionPoly345& other);
    ~ChFunctionPoly345() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionPoly345* Clone() const override { return new ChFunctionPoly345(*this); }

    virtual Type GetType() const override { return ChFunction::Type::POLY345; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;
    virtual double GetDer3(double x) const override;

    void SetWidth(double width);

    double GetWidth() const { return m_width; }

    void SetHeight(double height) { m_height = height; }

    double GetHeight() const { return m_height; }

    virtual double GetPositiveAccelerationCoeff() const override { return 5.8; }
    virtual double GetNegativeAccelerationCoeff() const override { return 5.8; }
    virtual double GetVelocityCoefficient() const override { return 1.9; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPoly345, 0)

}  // end namespace chrono

#endif
