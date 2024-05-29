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
// Authors: Alessandro Tasora, Radu Serban, Dario Mangoni
// =============================================================================

#ifndef CHFUNCT_INTERP_H
#define CHFUNCT_INTERP_H

#include <iterator>
#include <map>

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interpolation function.
/// Linear interpolation `y=f(x)` given a list of points `(x,y)`.
class ChApi ChFunctionInterp : public ChFunction {
  private:
    std::map<double, double> m_table;                                 ///< map with x-y points
    mutable std::map<double, double>::const_iterator m_last_greater;  ///< pointer to element greater than last 'x'
    bool m_extrapolate = false;  ///< enable linear extrapolation for out-of-range values

  public:
    ChFunctionInterp() : m_last_greater(m_table.end()), m_extrapolate(false) {}
    ChFunctionInterp(const ChFunctionInterp& other);
    ~ChFunctionInterp() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionInterp* Clone() const override { return new ChFunctionInterp(*this); }

    virtual Type GetType() const override { return ChFunction::Type::INTERP; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    /// Add a point to the table.
    /// By default, adding a point with an \a x value that already exists in the table will lead to an exception.
    /// If \a overwrite_if_existing is set to \c true, the existing point will be overwritten instead.
    void AddPoint(double x, double y, bool overwrite_if_existing = false);

    void Reset() {
        m_table.clear();
        m_last_greater = m_table.end();
    }

    /// Retrieve the underlying table of points.
    const std::map<double, double>& GetTable() { return m_table; }

    /// Return the smallest value of x in the table.
    double GetStart() const { return m_table.begin()->first; }

    /// Return the biggest value of x in the table.
    double GetEnd() const { return m_table.rbegin()->first; }

    /// Return the maximum function value in the table.
    double GetMax() const;

    /// Return the minimum function value in the table.
    double GetMin() const;

    /// Enable linear extrapolation.
    /// If enabled, the function will return linear extrapolation for \a x values outside the domain.
    /// while the first derivative will be kept equal to the derivative of the nearest two points.
    /// Second derivative in any case will be computed numerically based on first derivative.
    void SetExtrapolate(bool extrapolate) { m_extrapolate = extrapolate; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionInterp, 0)

}  // end namespace chrono

#endif
