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

#ifndef CHFUNCT_OSCILLOSCOPE_H
#define CHFUNCT_OSCILLOSCOPE_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Oscilloscope function
///
/// y = interpolation of array of (x,y) data,
///     where (x,y) points must be inserted one
///     after the other, strictly with a fixed dx
///     interval. After a maximum amount of recordable
///     points is reached, the firsts are deleted.
/// Note: differently from ChFunction_Recorder, this function does not allow
/// not-uniform dx spacing between points, but may be faster and simpler to
/// use in many cases.
class ChApi ChFunction_Oscilloscope : public ChFunction {
  private:
    std::list<double> values;
    double end_x;
    double dx;
    int max_amount;
    int amount;

  public:
    ChFunction_Oscilloscope() : end_x(0), dx(0.01), max_amount(100), amount(0) {}
    ChFunction_Oscilloscope(const ChFunction_Oscilloscope& other);
    ~ChFunction_Oscilloscope() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Oscilloscope* Clone() const override { return new ChFunction_Oscilloscope(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_OSCILLOSCOPE; }

    virtual double Get_y(double x) const override;

    /// Add a point at the head (right side of point array).
    /// Note that it is user's responsibility to add points
    /// which are spaced uniformly (by dx) on the X axis!
    /// No checks are done on the correctness of the dx spacing,
    /// except that if you enter a point whose mx is less than the
    /// mx of the one you previously entered, the array is cleared.
    void AddLastPoint(double mx, double my);

    /// Reset the array or recorded points.
    void Reset() {
        values.clear();
        amount = 0;
        end_x = 0;
    }

    /// Access directly the list of points.
    std::list<double>& GetPointList() { return values; }

    /// Get the dx spacing between recorded points. It is assumed uniform!
    double Get_dx() const { return dx; }
    /// Set the dx spacing between recorded points. It is assumed uniform!
    void Set_dx(double mdx) { dx = fabs(mdx); }

    /// Get the maximum amount of points which can be entered (after this,
    /// the first one will be deleted, as in a FIFO)
    int Get_max_amount() const { return max_amount; }
    /// Set the maximum amount of points which can be entered (after this,
    /// the first one will be deleted, as in a FIFO)
    void Set_max_amount(int mnum) {
        if (mnum > 0)
            max_amount = mnum;
    }

    /// Get the amount of recorded points
    double Get_amount() const { return amount; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_Oscilloscope, 0)

}  // end namespace chrono

#endif
