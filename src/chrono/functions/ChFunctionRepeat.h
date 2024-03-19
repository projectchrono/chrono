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

#ifndef CHFUNCT_REPEAT_H
#define CHFUNCT_REPEAT_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Repeat function:
///     `y = __/__/__/`
///
/// Repeats a slice of a function, periodically.
/// Returns:
///
///   f_repeat(slice_start + mod(t + slice_shift, slice_width))
///
/// Note: for infinite slice_width and zero slice_start, you can use
/// slice_shift to simply 'translate' the function on abscissa.
class ChApi ChFunctionRepeat : public ChFunction {
  public:
    ChFunctionRepeat(std::shared_ptr<ChFunction> func, double start = 0, double length = 1, double phase = 0);
    ChFunctionRepeat(const ChFunctionRepeat& other);
    ~ChFunctionRepeat() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionRepeat* Clone() const override { return new ChFunctionRepeat(*this); }

    virtual Type GetType() const override { return ChFunction::Type::REPEAT; }

    virtual double GetVal(double x) const override;

    /// Set the starting point of the slice that should be repeated.
    void SetSliceStart(double start) { m_slice_start = start; }

    /// Get the starting point of the slice that should be repeated.
    double GetSliceStart() const { return m_slice_start; }

    /// Set the width of the slice that should be repeated.
    void SetSliceWidth(double length) { m_slice_width = length; }

    /// Get the width of the slice that should be repeated.
    double GetSliceWidth() const { return m_slice_width; }

    /// Set the phase shift of the slice that should be repeated.
    void SetSliceShift(double phase) { m_slice_shift = phase; }

    /// Get the phase shift of the slice that should be repeated.
    double GetSliceShift() const { return m_slice_shift; }

    /// Set the function to be repeated.
    void SetRepeatedFunction(std::shared_ptr<ChFunction> func) { fa = func; }

    /// Get the function to be repeated.
    std::shared_ptr<ChFunction> GetRepeatedFunction() { return fa; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    double m_slice_start;  ///< slice start position
    double m_slice_width;  ///< slice width
    double m_slice_shift;  ///< slice shift
    std::shared_ptr<ChFunction> fa;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRepeat, 0)

}  // end namespace chrono

#endif
