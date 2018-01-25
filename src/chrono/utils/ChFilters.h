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
// Authors: Radu Serban
// =============================================================================
//
// A collection of various data filters
//
// =============================================================================

#ifndef CH_FILTERS_H
#define CH_FILTERS_H

#include <valarray>

#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Chrono core utilities
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Moving average filter for smoothing running data.
class ChApi ChRunningAverage {
  public:
    /// Construct a running moving average filter (backward).
    /// The filter only uses previous data (as specified by the filter span)
    ChRunningAverage(int n  ///< filter span
                     );

    ~ChRunningAverage() {}

    /// Insert the specified value and return the current moving average.
    double Add(double val);

    /// Return the standard deviation of data in current filter span.
    double GetStdDev() const { return m_std; }

    /// Reset the filter.
    void Reset();

  private:
    int m_n;
    int m_index;
    double m_std;
    std::valarray<double> m_data;
};

/// Moving average filter for smoothing a data array.
class ChApi ChMovingAverage {
  public:
    /// Construct a moving average filter (centered).
    /// The average is calculated over 2*n+1 points
    ChMovingAverage(const std::valarray<double>& data,  ///< input data
                    int n                               ///< filter half-span
                    );

    ~ChMovingAverage() {}

    /// Return the filtered output.
    const std::valarray<double>& Get() const { return m_out; }

    /// Return the specified component of the filtered output.
    double Get(int i) const { return m_out[i]; }

  private:
    int m_n;
    std::valarray<double> m_out;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
