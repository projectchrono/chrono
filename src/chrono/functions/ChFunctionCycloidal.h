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

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Cycloidal step function.
///     `y = h * (x/w - sin(2*pi * x/w)/2*pi)`
/// where:
/// - `h` is the height of the step
/// - `w` is the width of the step
class ChApi ChFunctionCycloidal : public ChFunction {
  public:
    ChFunctionCycloidal() : m_height(1), m_width(1) {}

    ChFunctionCycloidal(double height, double width);

    ChFunctionCycloidal(const ChFunctionCycloidal& other);

    ~ChFunctionCycloidal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionCycloidal* Clone() const override { return new ChFunctionCycloidal(*this); }

    virtual Type GetType() const override { return ChFunction::Type::CYCLOIDAL; }

    virtual double GetVal(double x) const override;

    virtual double GetDer(double x) const override;

    virtual double GetDer2(double x) const override;

    virtual double GetDer3(double x) const override;

    void SetWidth(double width);

    double GetWidth() const { return m_width; }

    void SetHeight(double height) { m_height = height; }

    double GetHeight() const { return m_height; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double m_height;  ///< final height of the step
    double m_width;   ///< width of the step
};

/// @} chrono_functions

}  // end namespace chrono

#endif
