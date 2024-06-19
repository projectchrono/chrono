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

#ifndef CHFUNCT_FILLET3_H
#define CHFUNCT_FILLET3_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Cubic fillet function.
/// Cubic polynomial `y(x)` that smoothly connects two points with given start and end derivatives.
/// - `y(0)`     = GetStartVal()
/// - `y(width)` = GetEndVal()
/// - `der(y)(0)` = GetStartDer()
/// - `der(y)(width)` = GetEndDer()
class ChApi ChFunctionFillet3 : public ChFunction {
  private:
    double m_width;      ///< width of the fillet
    double m_val_start;  ///< value at x=0
    double m_val_end;    ///< value at x=duration
    double m_der_start;  ///< derivative at x=0
    double m_der_end;    ///< derivative at x=m_width

    double c1, c2, c3, c4;  // used internally...

  public:
    ChFunctionFillet3()
        : m_width(1), m_val_start(0), m_val_end(0), m_der_start(0), m_der_end(0), c1(0), c2(0), c3(0), c4(0) {}
    ChFunctionFillet3(const ChFunctionFillet3& other);
    ~ChFunctionFillet3() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionFillet3* Clone() const override { return new ChFunctionFillet3(*this); }

    virtual Type GetType() const override { return ChFunction::Type::FILLET3; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    void SetWidth(double width);

    double GetWidth() { return m_width; }

    /// Setup the function after parameter changes.
    /// This must be called by the user after the parameters are changed.
    void Setup();

    /// Set the initial value of the function
    void SetStartVal(double val_start) { m_val_start = val_start; }

    /// Set the end value of the function
    void SetEndVal(double val_end) { m_val_end = val_end; }

    /// Set the initial derivative of the function
    void SetStartDer(double der_start) { m_der_start = der_start; }

    /// Set the end derivative of the function
    void SetEndDer(double der_end) { m_der_end = der_end; }

    /// Get the initial value of the function
    double GetStartVal() { return m_val_start; }

    /// Get the end value of the function
    double GetEndVal() { return m_val_end; }

    /// Get the initial derivative of the function
    double GetStartDer() { return m_der_start; }

    /// Get the end derivative of the function
    double GetEndDer() { return m_der_end; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

}  // namespace chrono

#endif
